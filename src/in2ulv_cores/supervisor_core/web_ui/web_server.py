#!/usr/bin/env python

import rospy
import threading
import time
import os
from flask import Flask, render_template
from flask_socketio import SocketIO
from msgs_core.msg import MonitoringMetric, AlertMessage

# 获取当前文件所在目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 创建 Flask 应用
app = Flask(__name__, 
            static_folder=os.path.join(current_dir, 'static'),
            template_folder=os.path.join(current_dir, 'templates'))
socketio = SocketIO(app, async_mode='threading')

# 存储监控数据
node_data = {}
topic_data = {}
alerts = []
lock = threading.Lock()

@app.route('/')
def index():
    """渲染监控面板主页"""
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    """当客户端连接时，发送当前所有数据"""
    with lock:
        socketio.emit('node_update', {'data': node_data})
        socketio.emit('topic_update', {'data': topic_data})
        socketio.emit('alert_update', {'data': alerts})

def metric_callback(msg):
    """处理监控指标回调"""
    with lock:
        # 处理节点状态
        if msg.metric_name == "node_status":
            node_data[msg.node_name] = {
                'status': msg.value,
                'timestamp': msg.stamp.to_sec()
            }
        # 处理话题频率
        elif "publish_frequency" in msg.metric_name:
            topic_data[msg.topic] = {
                'publisher': msg.node_name,
                'frequency': msg.value,
                'timestamp': msg.stamp.to_sec()
            }
        
        # 通过WebSocket发送更新
        socketio.emit('metric_update', {
            'node_name': msg.node_name,
            'topic': msg.topic,
            'metric_name': msg.metric_name,
            'value': msg.value
        })

def alert_callback(msg):
    """处理告警回调"""
    with lock:
        # 添加新告警
        severity_str = {
            AlertMessage.INFO: "info",
            AlertMessage.WARNING: "warning",
            AlertMessage.ERROR: "error"
        }.get(msg.severity, "unknown")
        
        alert = {
            'severity': severity_str,
            'source': msg.source,
            'message': msg.message,
            'timestamp': rospy.Time.now().to_sec()
        }
        
        alerts.append(alert)
        
        # 只保留最近的100条告警
        if len(alerts) > 100:
            alerts.pop(0)
        
        # 通过WebSocket发送新告警
        socketio.emit('new_alert', alert)

def run_web_server():
    """启动Web服务器"""
    rospy.loginfo("Starting web server on port 5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)

if __name__ == '__main__':
    rospy.init_node('web_ui_server', anonymous=True)
    
    # 订阅ROS话题
    rospy.Subscriber("/monitoring_metrics", MonitoringMetric, metric_callback)
    rospy.Subscriber("/system_alerts", AlertMessage, alert_callback)
    
    # 在单独线程中启动Web服务器
    web_thread = threading.Thread(target=run_web_server)
    web_thread.daemon = True
    web_thread.start()
    
    rospy.spin()