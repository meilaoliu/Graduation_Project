#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspection_dashboard / dashboard_node.py

Web 仪表盘后端：
  * Flask + Flask-SocketIO
  * 同进程内启动 ROS 节点 (rospy)，将 ROS 状态推到浏览器
  * 浏览器输入的指令通过 /chat_in 转回 nlp_commander

与 B样条 / MINCO 局部规划完全无关，仅消费高层话题。
"""

import base64
import os
import threading
import time

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    import cv2
    HAVE_CV = True
except Exception as _e:
    HAVE_CV = False
    _cv_err = str(_e)

try:
    from battery_simulator.msg import BatteryState
    HAVE_BATTERY = True
except Exception:
    HAVE_BATTERY = False

try:
    from inspection_services.msg import PhotoEvent
    HAVE_PHOTO = True
except Exception:
    HAVE_PHOTO = False

from flask import Flask, render_template
from flask_socketio import SocketIO


# ---------------------------------------------------------------------------
# 全局状态
# ---------------------------------------------------------------------------
class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.velocity = {'v': 0.0, 'w': 0.0}
        self.battery = {
            'percentage': 100.0,
            'charging': False,
            'status': 'ok',
            'estimated_remaining_distance_m': 0.0,
        }
        self.last_chat = []          # 仅用于刚连接时的回灌
        self.last_photo = None       # 最近一次拍照事件
        self.connected_clients = 0

    def snapshot_state(self):
        with self.lock:
            return {
                'position': dict(self.position),
                'velocity': dict(self.velocity),
                'battery': dict(self.battery),
                'time': time.time(),
            }


STATE = DashboardState()


# ---------------------------------------------------------------------------
# Flask + SocketIO
# ---------------------------------------------------------------------------
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATE_DIR = os.path.join(PKG_DIR, 'templates')
STATIC_DIR = os.path.join(PKG_DIR, 'static')
# 当通过 catkin install 运行时, scripts/ 与 templates/ 不在同一上层；做个兜底
if not os.path.isdir(TEMPLATE_DIR):
    alt = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'templates')
    if os.path.isdir(alt):
        TEMPLATE_DIR = os.path.abspath(alt)
        STATIC_DIR = os.path.abspath(os.path.join(alt, '..', 'static'))

app = Flask(__name__, template_folder=TEMPLATE_DIR, static_folder=STATIC_DIR)
app.config['SECRET_KEY'] = 'inspection-dashboard'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/healthz')
def healthz():
    return 'ok', 200


@app.route('/map')
def map_editor_page():
    return render_template('map_editor.html')


# ---------------------------------------------------------------------------
# OSM map editor
# ---------------------------------------------------------------------------
import xml.etree.ElementTree as ET
from xml.dom import minidom
from flask import request, jsonify, abort
import shutil
import datetime as _dt


def _osm_path():
    """从 rosparam 获取 osm 路径；默认指向 nlp_commander/maps/substation.osm"""
    p = rospy.get_param('~osm_path', '')
    if p and os.path.exists(p):
        return p
    # 默认相对路径回退
    try:
        import rospkg
        nlp_share = rospkg.RosPack().get_path('nlp_commander')
        cand = os.path.join(nlp_share, 'maps', 'substation.osm')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    # 兜底：源码中常见位置
    cand = os.path.expanduser(
        '~/Graduation_Project/ego-planner-for-ground-robot/src/'
        'nlp_commander/maps/substation.osm')
    return cand


def _parse_osm_to_payload(path):
    tree = ET.parse(path)
    root = tree.getroot()
    nodes, edges = [], []
    for nd in root.findall('node'):
        nid = nd.get('id')
        lat = nd.get('lat')
        lon = nd.get('lon')
        tags = {t.get('k'): t.get('v') for t in nd.findall('tag')}
        try:
            x = float(lon) if lon is not None else 0.0
            y = float(lat) if lat is not None else 0.0
        except ValueError:
            x, y = 0.0, 0.0
        nodes.append({
            'id': str(nid),
            'name': tags.get('name', f'node_{nid}'),
            'x': x,
            'y': y,
            'tags': {k: v for k, v in tags.items() if k != 'name'},
        })
    for w in root.findall('way'):
        wid = w.get('id')
        refs = [n.get('ref') for n in w.findall('nd') if n.get('ref')]
        if len(refs) < 2:
            continue
        wtags = {t.get('k'): t.get('v') for t in w.findall('tag')}
        # 单段 way (本项目约定)：仅取首尾
        edges.append({
            'id': str(wid),
            'a': str(refs[0]),
            'b': str(refs[-1]),
            'name': wtags.get('name', ''),
        })
    return {'osm_path': path, 'nodes': nodes, 'edges': edges}


def _write_osm_payload(path, payload):
    """把前端传回的 nodes/edges 重新写回 osm 文件 (带备份)。"""
    nodes = payload.get('nodes', [])
    edges = payload.get('edges', [])

    # 读原文件以保留 root attrs / 注释 (注释另行注入)
    try:
        orig_root = ET.parse(path).getroot()
        gen = orig_root.get('generator', 'manual')
    except Exception:
        gen = 'manual'

    osm = ET.Element('osm', {'version': '0.6', 'generator': gen})

    for n in nodes:
        attrs = {'id': str(n['id']),
                 'lat': str(n.get('y', 0.0)),
                 'lon': str(n.get('x', 0.0))}
        e = ET.SubElement(osm, 'node', attrs)
        # name 第一
        if n.get('name'):
            ET.SubElement(e, 'tag', {'k': 'name', 'v': str(n['name'])})
        for k, v in (n.get('tags') or {}).items():
            if k == 'name':
                continue
            ET.SubElement(e, 'tag', {'k': str(k), 'v': str(v)})

    for ed in edges:
        wattrs = {'id': str(ed['id'])}
        we = ET.SubElement(osm, 'way', wattrs)
        ET.SubElement(we, 'nd', {'ref': str(ed['a'])})
        ET.SubElement(we, 'nd', {'ref': str(ed['b'])})
        if ed.get('name'):
            ET.SubElement(we, 'tag', {'k': 'name', 'v': str(ed['name'])})

    # pretty-print
    rough = ET.tostring(osm, encoding='utf-8')
    pretty = minidom.parseString(rough).toprettyxml(indent='  ', encoding='utf-8')

    # 备份 + 原子写
    if os.path.exists(path):
        bak = path + '.bak.' + _dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        shutil.copy2(path, bak)
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(pretty)
    os.replace(tmp, path)
    return True


@app.route('/api/map', methods=['GET'])
def api_map_get():
    p = _osm_path()
    if not os.path.exists(p):
        return jsonify({'error': f'osm not found: {p}'}), 404
    try:
        return jsonify(_parse_osm_to_payload(p))
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/map', methods=['POST'])
def api_map_post():
    payload = request.get_json(silent=True) or {}
    if 'nodes' not in payload or 'edges' not in payload:
        return jsonify({'error': 'missing nodes/edges'}), 400
    target = payload.get('osm_path') or _osm_path()
    # 安全：只允许写已存在的目标路径或其同目录
    target = os.path.abspath(target)
    parent = os.path.dirname(target)
    default = os.path.abspath(_osm_path())
    if not (target == default or parent == os.path.dirname(default)):
        return jsonify({'error': 'osm_path not allowed'}), 403
    try:
        _write_osm_payload(target, payload)
        rospy.loginfo(f"[dashboard] osm saved: {target} "
                      f"(nodes={len(payload['nodes'])}, edges={len(payload['edges'])})")
        return jsonify({'ok': True, 'path': target})
    except Exception as e:
        rospy.logerr(f"[dashboard] osm save failed: {e}")
        return jsonify({'error': str(e)}), 500


@app.route('/api/robot_pose', methods=['GET'])
def api_robot_pose():
    snap = STATE.snapshot_state()
    p = snap['position']
    # 与 osm 约定一致：x=lon, y=lat
    return jsonify({'x': p['x'], 'y': p['y'], 'yaw': p['yaw']})


# ---------------------------------------------------------------------------
# ROS callbacks
# ---------------------------------------------------------------------------
_bridge = CvBridge() if HAVE_CV else None
_camera_last_emit = [0.0]
_camera_min_period = [0.2]   # 5Hz
_camera_jpeg_quality = [60]


def _yaw_from_quat(qx, qy, qz, qw):
    import math
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_cb(msg: Odometry):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    tw = msg.twist.twist
    yaw = _yaw_from_quat(o.x, o.y, o.z, o.w)
    with STATE.lock:
        STATE.position.update({'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw})
        STATE.velocity.update({'v': float(tw.linear.x), 'w': float(tw.angular.z)})


def battery_cb(msg):
    with STATE.lock:
        STATE.battery.update({
            'percentage': float(msg.percentage),
            'charging': bool(msg.charging),
            'status': str(msg.status),
            'estimated_remaining_distance_m': float(msg.estimated_remaining_distance_m),
        })


def chat_out_cb(msg: String):
    payload = {'role': 'robot', 'text': msg.data, 'time': time.time()}
    with STATE.lock:
        STATE.last_chat.append(payload)
        STATE.last_chat[:] = STATE.last_chat[-50:]
    socketio.emit('chat_msg', payload)


def photo_cb(msg):
    payload = {
        'label': msg.label,
        'filepath': msg.filepath,
        'thumb': msg.thumbnail_b64,
        'time': time.time(),
    }
    with STATE.lock:
        STATE.last_photo = payload
    socketio.emit('photo', payload)


def camera_cb(msg: Image):
    if not HAVE_CV:
        return
    if STATE.connected_clients <= 0:
        return                       # 没人看就不编码，省 CPU
    now = time.time()
    if now - _camera_last_emit[0] < _camera_min_period[0]:
        return
    _camera_last_emit[0] = now
    try:
        cv_img = _bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"[dashboard] cv_bridge error: {e}")
        return
    ok, buf = cv2.imencode('.jpg', cv_img,
                           [int(cv2.IMWRITE_JPEG_QUALITY), _camera_jpeg_quality[0]])
    if not ok:
        return
    b64 = base64.b64encode(buf.tobytes()).decode('ascii')
    socketio.emit('camera_frame', {'jpeg_b64': b64, 'time': now})


# ---------------------------------------------------------------------------
# SocketIO events
# ---------------------------------------------------------------------------
_chat_in_pub = None


@socketio.on('connect')
def on_connect():
    with STATE.lock:
        STATE.connected_clients += 1
        history = list(STATE.last_chat)
        last_photo = STATE.last_photo
    socketio.emit('hello', {'ok': True})
    if history:
        socketio.emit('chat_history', history)
    if last_photo:
        socketio.emit('photo', last_photo)


@socketio.on('disconnect')
def on_disconnect():
    with STATE.lock:
        STATE.connected_clients = max(0, STATE.connected_clients - 1)


@socketio.on('user_cmd')
def on_user_cmd(data):
    text = (data or {}).get('text', '').strip()
    if not text:
        return
    payload = {'role': 'user', 'text': text, 'time': time.time()}
    with STATE.lock:
        STATE.last_chat.append(payload)
        STATE.last_chat[:] = STATE.last_chat[-50:]
    socketio.emit('chat_msg', payload)
    if _chat_in_pub is not None:
        try:
            _chat_in_pub.publish(String(data=text))
        except Exception as e:
            rospy.logwarn(f"[dashboard] /chat_in publish failed: {e}")


# ---------------------------------------------------------------------------
# 状态心跳
# ---------------------------------------------------------------------------
def state_pusher():
    rate_hz = 1.0
    period = 1.0 / rate_hz
    while not rospy.is_shutdown():
        try:
            if STATE.connected_clients > 0:
                socketio.emit('state_update', STATE.snapshot_state())
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[dashboard] state push failed: {e}")
        time.sleep(period)


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    global _chat_in_pub

    rospy.init_node('inspection_dashboard', anonymous=False, disable_signals=True)

    host = rospy.get_param('~host', '0.0.0.0')
    port = int(rospy.get_param('~port', 5000))
    odom_topic = rospy.get_param('~odom_topic', '/odom_adjust')
    image_topic = rospy.get_param('~image_topic', '/camera/image')
    _camera_min_period[0] = 1.0 / float(rospy.get_param('~camera_max_hz', 5.0))
    _camera_jpeg_quality[0] = int(rospy.get_param('~camera_jpeg_quality', 60))

    rospy.Subscriber(odom_topic, Odometry, odom_cb, queue_size=20)
    rospy.Subscriber('/chat_out', String, chat_out_cb, queue_size=50)
    if HAVE_BATTERY:
        rospy.Subscriber('/battery_state', BatteryState, battery_cb, queue_size=10)
    else:
        rospy.logwarn("[dashboard] battery_simulator msgs not available — battery panel disabled")
    if HAVE_PHOTO:
        rospy.Subscriber('/photo_event', PhotoEvent, photo_cb, queue_size=20)
    else:
        rospy.logwarn("[dashboard] inspection_services msgs not available — photo panel disabled")
    if HAVE_CV:
        rospy.Subscriber(image_topic, Image, camera_cb, queue_size=1, buff_size=2 ** 22)
    else:
        rospy.logwarn(f"[dashboard] cv_bridge/cv2 unavailable, camera disabled: {_cv_err}")

    _chat_in_pub = rospy.Publisher('/chat_in', String, queue_size=10)

    threading.Thread(target=state_pusher, daemon=True).start()

    rospy.loginfo(f"[dashboard] Serving on http://{host}:{port}  (odom={odom_topic}, image={image_topic})")
    try:
        socketio.run(app, host=host, port=port, debug=False,
                     use_reloader=False, allow_unsafe_werkzeug=True)
    except TypeError:
        # 老版 flask-socketio 不认识 allow_unsafe_werkzeug 参数
        socketio.run(app, host=host, port=port, debug=False, use_reloader=False)
    finally:
        rospy.signal_shutdown('flask exited')


if __name__ == '__main__':
    main()
