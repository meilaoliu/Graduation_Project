#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
photo_service_node
==================
- 订阅 /camera/image (sensor_msgs/Image，CMU vehicleSimulator 默认 320x180@15Hz)
- 缓存最新一帧；提供 /take_photo 服务保存图片并广播 /photo_event
- 与 B 样条 / MINCO 规划器无关：纯传感器侧消费节点
"""

import os
import time
import base64
import threading

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from inspection_services.srv import TakePhoto, TakePhotoResponse
from inspection_services.msg import PhotoEvent


class PhotoService:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()

        self.image_topic = rospy.get_param('~image_topic', '/camera/image')
        self.save_dir = os.path.expanduser(rospy.get_param('~save_dir', '~/inspection_photos'))
        self.thumb_max_w = int(rospy.get_param('~thumbnail_max_width', 240))
        self.jpeg_quality = int(rospy.get_param('~jpeg_quality', 90))
        os.makedirs(self.save_dir, exist_ok=True)

        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.event_pub = rospy.Publisher('/photo_event', PhotoEvent, queue_size=10, latch=False)
        self.srv = rospy.Service('/take_photo', TakePhoto, self.handle_take_photo)

        rospy.loginfo("[photo_service] ready. image_topic=%s, save_dir=%s",
                      self.image_topic, self.save_dir)

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[photo_service] cv_bridge failed: {e}")
            return
        with self.lock:
            self.latest_frame = cv_img

    def handle_take_photo(self, req):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()

        if frame is None:
            return TakePhotoResponse(success=False, filepath="",
                                     message="no camera frame yet")

        label = req.label or "photo"
        safe_label = "".join(c if c.isalnum() or c in '-_' else '_' for c in label)
        ts = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{ts}_{safe_label}.jpg"
        filepath = os.path.join(self.save_dir, filename)

        try:
            cv2.imwrite(filepath, frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        except Exception as e:
            return TakePhotoResponse(success=False, filepath="", message=f"imwrite failed: {e}")

        thumb_b64 = self._encode_thumbnail(frame)

        evt = PhotoEvent()
        evt.header = Header(stamp=rospy.Time.now())
        evt.label = label
        evt.filepath = filepath
        evt.thumbnail_b64 = thumb_b64
        self.event_pub.publish(evt)

        rospy.loginfo("[photo_service] saved %s", filepath)
        return TakePhotoResponse(success=True, filepath=filepath, message="ok")

    def _encode_thumbnail(self, frame):
        try:
            h, w = frame.shape[:2]
            if w > self.thumb_max_w:
                new_w = self.thumb_max_w
                new_h = int(h * (new_w / w))
                frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                return ""
            return base64.b64encode(buf.tobytes()).decode('ascii')
        except Exception:
            return ""


def main():
    rospy.init_node('photo_service_node')
    PhotoService()
    rospy.spin()


if __name__ == '__main__':
    main()
