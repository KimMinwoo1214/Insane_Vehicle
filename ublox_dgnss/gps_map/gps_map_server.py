#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template
from flask_socketio import SocketIO
import utm

app = Flask(__name__)
# 모든 오리진 허용, 브로드캐스트 모드 사용
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins='*')

@app.route('/')
def index():
    return render_template('index.html')

class GPSMapNode(Node):
    def __init__(self):
        super().__init__('gps_map_node')
        self.sub = self.create_subscription(String, '/gps', self.gps_callback, 10)

    def gps_callback(self, msg: String):
        try:
            nah, nah , head_mot, lat_str, lon_str = msg.data.split(',')
            data = {'lat': float(lat_str), 'lon': float(lon_str)}
            # 모든 클라이언트에게 전송
            socketio.emit('new_coord', data)
        except Exception as e:
            lon_str, lat_str = msg.data.split(',')
            data = {'lat': float(lat_str), 'lon': float(lon_str)}

def start_ros_spin(node):
    rclpy.spin(node)

if __name__ == '__main__':
    rclpy.init()
    node = GPSMapNode()

    # ROS 스핀을 데몬 스레드로 실행
    t = threading.Thread(target=start_ros_spin, args=(node,), daemon=True)
    t.start()

    socketio.run(app, host='0.0.0.0', port=5000, debug=False)

    node.destroy_node()
    rclpy.shutdown()
