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
        self.last_photo = None       # 最近一次拍照事件 (兼容字段, 弃用中)
        self.photos = []             # 最近 N 次拍照事件，刷新时回灌
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


def _set_or_replace_tag(elem, k, v):
    """在 elem 下找 <tag k=k>，更新 v；不存在则新增。"""
    for t in elem.findall('tag'):
        if t.get('k') == k:
            if t.get('v') != v:
                t.set('v', v)
            return False  # not added
    ET.SubElement(elem, 'tag', {'k': k, 'v': v})
    return True


def _remove_extra_tags(elem, keep_keys):
    """删除 elem 下不在 keep_keys 里的 <tag>。"""
    for t in list(elem.findall('tag')):
        if t.get('k') not in keep_keys:
            elem.remove(t)


def _indent_inplace(elem, level=0, step='  '):
    """给单个新增 element 加缩进文本（只影响它自己和子节点，不动兄弟）。"""
    i = '\n' + level * step
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + step
        for child in elem:
            _indent_inplace(child, level + 1, step)
            if not child.tail or not child.tail.strip():
                child.tail = i + step
        if elem[-1].tail:
            elem[-1].tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def _coord_eq(old_str, new_val):
    """判断坐标字符串和 float 是否等价（避免把 '27' 改成 '27.0'）。"""
    if old_str is None:
        return False
    try:
        return abs(float(old_str) - float(new_val)) < 1e-9
    except Exception:
        return False


def _write_osm_payload(path, payload):
    """就地更新 osm：只改用户实际修改的 lat/lon/tag，最小化 diff。

    保留原文件的注释、空行、属性顺序、XML declaration 大小写。
    新节点/边追加到末尾；被删除的节点/边从 tree 移除。
    """
    nodes = payload.get('nodes', [])
    edges = payload.get('edges', [])

    if os.path.exists(path):
        # insert_comments=True 让 <!-- ... --> 被保留为 ET.Comment 节点
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        tree = ET.parse(path, parser=parser)
        root = tree.getroot()
    else:
        root = ET.Element('osm', {'version': '0.6', 'generator': 'manual'})
        tree = ET.ElementTree(root)

    # 建索引（跳过 Comment）
    node_by_id = {n.get('id'): n for n in root.findall('node')}
    way_by_id = {w.get('id'): w for w in root.findall('way')}

    seen_node_ids = set()
    seen_way_ids = set()

    for n in nodes:
        nid = str(n['id'])
        seen_node_ids.add(nid)
        new_lat = n.get('y', 0.0)
        new_lon = n.get('x', 0.0)
        tags = dict(n.get('tags') or {})
        if n.get('name'):
            tags['name'] = str(n['name'])

        elem = node_by_id.get(nid)
        if elem is None:
            elem = ET.SubElement(root, 'node',
                                 {'id': nid, 'lat': str(new_lat), 'lon': str(new_lon)})
            for k, v in tags.items():
                ET.SubElement(elem, 'tag', {'k': str(k), 'v': str(v)})
            _indent_inplace(elem, level=1)
        else:
            # 数值不变时保留原始字符串格式（'27' 不变成 '27.0'）
            if not _coord_eq(elem.get('lat'), new_lat):
                elem.set('lat', str(new_lat))
            if not _coord_eq(elem.get('lon'), new_lon):
                elem.set('lon', str(new_lon))
            for k, v in tags.items():
                _set_or_replace_tag(elem, str(k), str(v))
            _remove_extra_tags(elem, set(map(str, tags.keys())))

    for nid, elem in node_by_id.items():
        if nid not in seen_node_ids:
            root.remove(elem)

    for ed in edges:
        wid = str(ed['id'])
        seen_way_ids.add(wid)
        a = str(ed['a']); b = str(ed['b'])
        name = str(ed['name']) if ed.get('name') else None

        elem = way_by_id.get(wid)
        if elem is None:
            elem = ET.SubElement(root, 'way', {'id': wid})
            ET.SubElement(elem, 'nd', {'ref': a})
            ET.SubElement(elem, 'nd', {'ref': b})
            if name:
                ET.SubElement(elem, 'tag', {'k': 'name', 'v': name})
            _indent_inplace(elem, level=1)
        else:
            nds = elem.findall('nd')
            new_refs = [a, b]
            old_refs = [nd.get('ref') for nd in nds]
            if old_refs != new_refs:
                tags_keep = list(elem.findall('tag'))
                for child in list(elem):
                    elem.remove(child)
                ET.SubElement(elem, 'nd', {'ref': a})
                ET.SubElement(elem, 'nd', {'ref': b})
                for t in tags_keep:
                    elem.append(t)
            if name:
                _set_or_replace_tag(elem, 'name', name)
                _remove_extra_tags(elem, {'name'})
            else:
                _remove_extra_tags(elem, set())

    for wid, elem in way_by_id.items():
        if wid not in seen_way_ids:
            root.remove(elem)

    # 序列化：自己控制属性引号(") 和自闭合标签格式(<tag .../>)，匹配原文件风格
    body = ET.tostring(root, encoding='unicode', short_empty_elements=True)
    # ET 默认输出 ` />`（带空格），原文件是 `/>`（无空格） → 修正
    body = body.replace(' />', '/>')
    out = '<?xml version="1.0" encoding="UTF-8"?>\n' + body + '\n'

    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(out)
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


def _simplified_osm_path(path):
    """与 substation.osm 同目录的 simplified_substation.osm"""
    d = os.path.dirname(path)
    return os.path.join(d, 'simplified_substation.osm')


def _write_simplified_osm(path, payload):
    """从带几何的 osm payload 生成简化版 (无 lat/lon, 无 way)，给 LLM 使用。"""
    osm = ET.Element('osm', {'version': '0.6', 'generator': 'simplified'})
    for n in payload.get('nodes', []):
        e = ET.SubElement(osm, 'node', {'id': str(n['id'])})
        if n.get('name'):
            ET.SubElement(e, 'tag', {'k': 'name', 'v': str(n['name'])})
        for k, v in (n.get('tags') or {}).items():
            if k == 'name':
                continue
            ET.SubElement(e, 'tag', {'k': str(k), 'v': str(v)})
    _indent_inplace(osm, level=0)
    body = ET.tostring(osm, encoding='unicode', short_empty_elements=True)
    body = body.replace(' />', '/>')
    out = '<?xml version="1.0" encoding="UTF-8"?>\n' + body + '\n'
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(out)
    os.replace(tmp, path)


def _call_reload_map_service(timeout=2.0):
    """尝试调用 nlp_commander 的 /reload_map 服务；返回 (ok, msg)。"""
    try:
        rospy.wait_for_service('/reload_map', timeout=timeout)
        proxy = rospy.ServiceProxy('/reload_map', __import__('std_srvs.srv', fromlist=['Trigger']).Trigger)
        resp = proxy()
        return bool(resp.success), str(resp.message)
    except Exception as e:
        return False, f'{e}'


@app.route('/api/map', methods=['POST'])
def api_map_post():
    payload = request.get_json(silent=True) or {}
    if 'nodes' not in payload or 'edges' not in payload:
        return jsonify({'error': 'missing nodes/edges'}), 400
    target = payload.get('osm_path') or _osm_path()
    target = os.path.abspath(target)
    parent = os.path.dirname(target)
    default = os.path.abspath(_osm_path())
    if not (target == default or parent == os.path.dirname(default)):
        return jsonify({'error': 'osm_path not allowed'}), 403
    try:
        # 1. 自动存档（如果旧文件存在），保留误操作回滚能力
        archive_msg = _archive_osm_if_exists(target)

        _write_osm_payload(target, payload)
        # 同步生成简化版 (供 LLM 使用)
        simp_path = _simplified_osm_path(target)
        try:
            _write_simplified_osm(simp_path, payload)
            simp_ok, simp_msg = True, simp_path
        except Exception as e:
            simp_ok, simp_msg = False, f'{e}'
            rospy.logwarn(f"[dashboard] simplified osm write failed: {e}")

        # 触发 nlp_commander 热加载
        reload_ok, reload_msg = _call_reload_map_service()

        rospy.loginfo(f"[dashboard] osm saved: {target} "
                      f"(nodes={len(payload['nodes'])}, edges={len(payload['edges'])}, "
                      f"simplified={'ok' if simp_ok else 'fail'}, "
                      f"reload={'ok' if reload_ok else 'fail'}, "
                      f"archive={archive_msg})")
        return jsonify({
            'ok': True,
            'path': target,
            'archive':    archive_msg,
            'simplified': {'ok': simp_ok, 'msg': simp_msg},
            'reload':     {'ok': reload_ok, 'msg': reload_msg},
        })
    except Exception as e:
        rospy.logerr(f"[dashboard] osm save failed: {e}")
        return jsonify({'error': str(e)}), 500


# ---------------------------------------------------------------------------
# OSM 历史存档（误操作回滚）
# ---------------------------------------------------------------------------
def _archive_dir():
    return os.path.join(os.path.dirname(_osm_path()), '_archive')


def _archive_osm_if_exists(target_path):
    if not os.path.isfile(target_path):
        return 'no-prior'
    try:
        d = _archive_dir()
        os.makedirs(d, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        base = os.path.basename(target_path)
        archive_name = f'{base}.{ts}.osm'
        archive_path = os.path.join(d, archive_name)
        # 简单 copy
        with open(target_path, 'rb') as fin, open(archive_path, 'wb') as fout:
            fout.write(fin.read())
        # 限制最多保留 30 份，按修改时间删旧的
        snaps = sorted(
            (f for f in os.listdir(d) if f.startswith(base + '.') and f.endswith('.osm')),
            key=lambda f: os.path.getmtime(os.path.join(d, f)),
        )
        for old in snaps[:-30]:
            try:
                os.remove(os.path.join(d, old))
            except Exception:
                pass
        return archive_name
    except Exception as e:
        rospy.logwarn(f"[dashboard] archive failed: {e}")
        return f'fail: {e}'


@app.route('/api/map/archives', methods=['GET'])
def api_map_archives():
    d = _archive_dir()
    items = []
    if os.path.isdir(d):
        base = os.path.basename(_osm_path())
        for name in os.listdir(d):
            if not (name.startswith(base + '.') and name.endswith('.osm')):
                continue
            path = os.path.join(d, name)
            try:
                st = os.stat(path)
                items.append({
                    'name': name,
                    'mtime': st.st_mtime,
                    'size': st.st_size,
                })
            except Exception:
                pass
    items.sort(key=lambda x: x['mtime'], reverse=True)
    return jsonify({'archives': items})


@app.route('/api/map/restore', methods=['POST'])
def api_map_restore():
    body = request.get_json(silent=True) or {}
    name = body.get('name', '')
    if not name or '/' in name or '\\' in name or '..' in name:
        return jsonify({'ok': False, 'error': 'invalid name'}), 400
    src = os.path.join(_archive_dir(), name)
    if not os.path.isfile(src):
        return jsonify({'ok': False, 'error': 'archive not found'}), 404
    target = _osm_path()
    try:
        # 把当前 osm 也存一档（防止恢复后又想回到刚才的状态）
        _archive_osm_if_exists(target)
        with open(src, 'rb') as fin, open(target, 'wb') as fout:
            fout.write(fin.read())
        # 解析回 payload，重生成简化版 + reload
        payload = _parse_osm_to_payload(target)
        try:
            _write_simplified_osm(_simplified_osm_path(target), payload)
        except Exception as e:
            rospy.logwarn(f"[dashboard] simplified write failed on restore: {e}")
        reload_ok, reload_msg = _call_reload_map_service()
        return jsonify({'ok': True, 'restored': name,
                        'reload': {'ok': reload_ok, 'msg': reload_msg}})
    except Exception as e:
        rospy.logerr(f"[dashboard] restore failed: {e}")
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/robot_pose', methods=['GET'])
def api_robot_pose():
    snap = STATE.snapshot_state()
    p = snap['position']
    # 与 osm 约定一致：x=lon, y=lat
    return jsonify({'x': p['x'], 'y': p['y'], 'yaw': p['yaw']})


# ---------------------------------------------------------------------------
# Basemap (overlay 真实俯视图，方便对齐拓扑)
# ---------------------------------------------------------------------------
import json as _json

_BASEMAP_DIR = os.path.join(STATIC_DIR, 'basemap')
_BASEMAP_CFG = os.path.join(_BASEMAP_DIR, '_config.json')
_BASEMAP_EXTS = ('.png', '.jpg', '.jpeg', '.webp', '.gif', '.svg')

_BASEMAP_DEFAULT = {
    'enabled': False,
    'source': 'local',          # 'local' | 'url'
    'src': '',                  # 文件名（local）或 URL
    'cx': 0.0,                  # 世界坐标系下底图中心 x
    'cy': 0.0,                  # 世界坐标系下底图中心 y
    'world_width': 100.0,       # 底图宽对应的世界单位（米）
    'rotation': 0.0,            # 顺时针角度
    'opacity': 0.5,
}


def _basemap_load_cfg():
    cfg = dict(_BASEMAP_DEFAULT)
    try:
        if os.path.isfile(_BASEMAP_CFG):
            with open(_BASEMAP_CFG, 'r', encoding='utf-8') as f:
                user = _json.load(f) or {}
            cfg.update({k: user[k] for k in cfg.keys() if k in user})
    except Exception as e:
        rospy.logwarn(f"[dashboard] basemap config load failed: {e}")
    return cfg


def _basemap_save_cfg(cfg):
    try:
        os.makedirs(_BASEMAP_DIR, exist_ok=True)
        merged = dict(_BASEMAP_DEFAULT)
        merged.update({k: cfg[k] for k in _BASEMAP_DEFAULT if k in cfg})
        with open(_BASEMAP_CFG, 'w', encoding='utf-8') as f:
            _json.dump(merged, f, ensure_ascii=False, indent=2)
        return merged
    except Exception as e:
        raise RuntimeError(f"basemap config save failed: {e}")


@app.route('/api/basemap/list', methods=['GET'])
def api_basemap_list():
    files = []
    if os.path.isdir(_BASEMAP_DIR):
        for name in sorted(os.listdir(_BASEMAP_DIR)):
            if name.startswith('_'):
                continue
            if name.lower().endswith(_BASEMAP_EXTS):
                files.append({
                    'name': name,
                    'url': f'/static/basemap/{name}',
                })
    return jsonify({'files': files})


@app.route('/api/basemap/config', methods=['GET'])
def api_basemap_config_get():
    return jsonify(_basemap_load_cfg())


@app.route('/api/basemap/config', methods=['POST'])
def api_basemap_config_post():
    body = request.get_json(silent=True) or {}
    try:
        merged = _basemap_save_cfg(body)
        return jsonify({'ok': True, 'config': merged})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 500


@app.route('/api/basemap/upload', methods=['POST'])
def api_basemap_upload():
    """允许从浏览器上传任意本地图片作为底图。

    支持两种姿势：
      A) multipart/form-data  字段 file=<文件>
      B) JSON                 {filename, data_url} (data_url 以 'data:image/...;base64,' 开头)
    上传后保存到 static/basemap/，文件名清洗后返回。
    """
    import re as _re
    import base64 as _b64
    saved_name = None
    saved_bytes = None

    # ---- 模式 A: multipart ----
    if 'file' in request.files:
        f = request.files['file']
        raw_name = (f.filename or '').strip()
        if not raw_name:
            return jsonify({'ok': False, 'error': 'empty filename'}), 400
        saved_bytes = f.read()
        saved_name = raw_name
    else:
        body = request.get_json(silent=True) or {}
        data_url = body.get('data_url') or ''
        raw_name = (body.get('filename') or '').strip()
        m = _re.match(r'^data:([^;]+);base64,(.+)$', data_url)
        if not (m and raw_name):
            return jsonify({'ok': False, 'error': 'expect multipart "file" or JSON {filename, data_url}'}), 400
        try:
            saved_bytes = _b64.b64decode(m.group(2))
        except Exception as e:
            return jsonify({'ok': False, 'error': f'bad base64: {e}'}), 400
        saved_name = raw_name

    # 清洗文件名 + 后缀白名单
    safe = _re.sub(r'[^A-Za-z0-9._\u4e00-\u9fa5-]', '_', saved_name)
    safe = safe.lstrip('._') or 'upload'
    base, ext = os.path.splitext(safe)
    if ext.lower() not in _BASEMAP_EXTS:
        return jsonify({'ok': False, 'error': f'unsupported ext: {ext}'}), 400

    try:
        os.makedirs(_BASEMAP_DIR, exist_ok=True)
        # 重名加后缀
        target = os.path.join(_BASEMAP_DIR, safe)
        if os.path.exists(target):
            ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
            safe = f'{base}_{ts}{ext}'
            target = os.path.join(_BASEMAP_DIR, safe)
        with open(target, 'wb') as fout:
            fout.write(saved_bytes)
        rospy.loginfo(f"[dashboard] basemap uploaded: {target} ({len(saved_bytes)} bytes)")
        return jsonify({
            'ok': True,
            'name': safe,
            'url': f'/static/basemap/{safe}',
            'size': len(saved_bytes),
        })
    except Exception as e:
        rospy.logerr(f"[dashboard] basemap upload failed: {e}")
        return jsonify({'ok': False, 'error': str(e)}), 500


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
        STATE.photos.append(payload)
        STATE.photos[:] = STATE.photos[-50:]
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
    h, w = cv_img.shape[:2]
    socketio.emit('camera_frame', {'jpeg_b64': b64, 'time': now, 'width': int(w), 'height': int(h)})


# ---------------------------------------------------------------------------
# SocketIO events
# ---------------------------------------------------------------------------
_chat_in_pub = None


@socketio.on('connect')
def on_connect():
    with STATE.lock:
        STATE.connected_clients += 1
        history = list(STATE.last_chat)
        photos = list(STATE.photos)
    socketio.emit('hello', {'ok': True})
    if history:
        socketio.emit('chat_history', history)
    if photos:
        socketio.emit('photo_history', photos)


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

    # 干净关闭：SIGINT/SIGTERM 时同时关 rospy 与 socketio,
    # 避免在 ROS master 里留下 stale 订阅，导致重启后接不到消息。
    import signal as _signal
    _shutting_down = {'flag': False}
    def _graceful(signum, _frame):
        if _shutting_down['flag']:
            return
        _shutting_down['flag'] = True
        rospy.loginfo(f"[dashboard] signal {signum} → shutting down (rospy + socketio)")
        try:
            rospy.signal_shutdown(f'signal {signum}')
        except Exception:
            pass
        try:
            socketio.stop()
        except Exception:
            pass
    _signal.signal(_signal.SIGINT, _graceful)
    _signal.signal(_signal.SIGTERM, _graceful)

    rospy.loginfo(f"[dashboard] Serving on http://{host}:{port}  (odom={odom_topic}, image={image_topic})")
    try:
        socketio.run(app, host=host, port=port, debug=False,
                     use_reloader=False, allow_unsafe_werkzeug=True)
    except TypeError:
        socketio.run(app, host=host, port=port, debug=False, use_reloader=False)
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown('flask exited')


if __name__ == '__main__':
    main()
