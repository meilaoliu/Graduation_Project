# inspection_dashboard

变电站巡检 Web 仪表盘 — Flask + Flask-SocketIO + ROS 桥接，单页 4 面板：

| 面板         | 内容                                                |
| ------------ | --------------------------------------------------- |
| 机器人状态   | 位置 / 朝向 / 线速 / 角速 / 电量条 / 状态 / 续航   |
| 实时画面     | `/camera/image` JPEG 5 Hz 推送                      |
| 对话框       | 双向 — 浏览器 → `/chat_in`，机器人 → `/chat_out`   |
| 拍照画廊     | 订阅 `/photo_event`，展示 thumbnail + 标签         |

## 与规划器后端的关系

本节点**不感知** B 样条 / MINCO 的具体实现，只消费高层话题：
`/odom_adjust`、`/battery_state`、`/photo_event`、`/chat_out`、`/camera/image`。
两种规划器后端均直接兼容。

## 依赖

```bash
pip3 install --user flask flask-socketio python-socketio
sudo apt install ros-noetic-cv-bridge python3-opencv     # 若未装
```

## 启动

```bash
roslaunch inspection_dashboard dashboard.launch
# 浏览器访问 http://<robot-host>:5000/
```

参数：

- `port` (默认 5000)
- `odom_topic` (默认 `/odom_adjust`)
- `image_topic` (默认 `/camera/image`)
- `camera_max_hz` (默认 5)
- `camera_jpeg_quality` (默认 60)

## 设计要点

- `socketio.async_mode='threading'`，与 `rospy` 回调线程模型兼容；状态心跳 1 Hz 推送。
- 没有客户端连接时不编码图像 (`STATE.connected_clients>0` 时才 emit) 节省 CPU。
- 浏览器输入通过 `std_msgs/String` 发到 `/chat_in`，与现有 `nlp_commander` stdin 输入并行。
