#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
from .web_server import WebMonitor

class WebMonitorNode(Node):
    def __init__(self):
        super().__init__('web_monitor_node')
        
        # 포트 파라미터 선언 및 가져오기
        self.declare_parameter('port', 5000)
        port = self.get_parameter('port').get_parameter_value().integer_value
        
        # 웹 모니터 인스턴스 생성
        self.web_monitor = WebMonitor(port=port)
        
        # ROS2 구독자 생성
        self.compression_ratio_sub = self.create_subscription(
            Float64,
            '/draco/compression_ratio',
            self.compression_ratio_callback,
            10
        )
        
        self.decompression_time_sub = self.create_subscription(
            Float64,
            '/draco/decompression_time',
            self.decompression_time_callback,
            10
        )
        
        self.network_throughput_sub = self.create_subscription(
            Float64,
            '/draco/network_throughput',
            self.network_throughput_callback,
            10
        )
        
        self.decompressed_size_sub = self.create_subscription(
            Float64,
            '/draco/decompressed_size',
            self.decompressed_size_callback,
            10
        )
        
        # 웹 서버를 별도 스레드에서 실행
        self.web_thread = threading.Thread(target=self.web_monitor.start)
        self.web_thread.daemon = True
        self.web_thread.start()
        
        self.get_logger().info('웹 모니터링 노드가 시작되었습니다.')
        self.get_logger().info(f'웹 인터페이스: http://localhost:{port}')
    
    def compression_ratio_callback(self, msg):
        """압축률 데이터 콜백"""
        self.web_monitor.metrics['compression_ratio'].append(msg.data)
    
    def decompression_time_callback(self, msg):
        """압축 해제 시간 데이터 콜백"""
        self.web_monitor.metrics['decompression_time'].append(msg.data)
    
    def network_throughput_callback(self, msg):
        """네트워크 처리량 데이터 콜백"""
        self.web_monitor.metrics['network_throughput'].append(msg.data)
    
    def decompressed_size_callback(self, msg):
        """압축 해제 후 데이터 크기 콜백"""
        self.web_monitor.metrics['decompressed_size'].append(msg.data)

def main(args=None):
    rclpy.init(args=args)
    
    node = WebMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
