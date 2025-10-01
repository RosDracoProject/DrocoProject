"""
LiDAR-Draco Bridge Module

Core bridge functionality for connecting LiDAR simulation data to Draco compression system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import socket
import json
import struct
import time
import threading
from typing import List, Dict, Any
import numpy as np

# point_cloud_transport_py 사용
from point_cloud_transport_py import PointCloudTransport


class LidarDracoBridge(Node):
    """
    LiDAR-Draco Bridge Node
    
    Bridges LiDAR simulation data to Draco compression system using point_cloud_transport_py.
    Supports efficient compression and transmission of point cloud data.
    """
    
    def __init__(self):
        super().__init__('lidar_draco_bridge')
        
        # ROS2 파라미터
        self.declare_parameter('draco_server_host', 'localhost')
        self.declare_parameter('draco_server_port', 8080)
        self.declare_parameter('lidar_topic', '/velodyne2/velodyne_points2')
        self.declare_parameter('output_topic', '/lidar/compressed')
        self.declare_parameter('compression_type', 'draco')
        self.declare_parameter('compression_level', 6)
        self.declare_parameter('max_clients', 10)
        self.declare_parameter('send_rate', 10.0)  # Hz
        
        # 파라미터 가져오기
        self.draco_host = self.get_parameter('draco_server_host').value
        self.draco_port = self.get_parameter('draco_server_port').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.compression_type = self.get_parameter('compression_type').value
        self.compression_level = self.get_parameter('compression_level').value
        self.max_clients = self.get_parameter('max_clients').value
        self.send_rate = self.get_parameter('send_rate').value
        
        # PointCloudTransport 초기화
        self.transport = PointCloudTransport(self)
        
        # 압축 설정
        if self.compression_type == 'draco':
            self.transport.set_compression_settings('draco', compression_level=self.compression_level)
        elif self.compression_type == 'zlib':
            self.transport.set_compression_settings('zlib', level=self.compression_level)
        
        # QoS 프로파일 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 압축된 포인트 클라우드 구독자 생성
        self.subscriber = self.transport.subscribe(
            self.lidar_topic,
            self.point_cloud_callback,
            qos_profile=qos_profile,
            transport=self.compression_type
        )
        
        # 압축된 포인트 클라우드 퍼블리셔 생성
        self.publisher = self.transport.advertise(
            self.output_topic,
            queue_size=1,
            qos_profile=qos_profile,
            transport=self.compression_type
        )
        
        # 통계 변수
        self.message_count = 0
        self.bytes_sent = 0
        self.start_time = time.time()
        self.clients: List[socket.socket] = []
        
        # 서버 소켓 설정
        self.server_socket = None
        self.server_thread = None
        self.running = False
        
        # 타이머 생성 (주기적 통계 출력)
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        # 드라코 서버 시작
        self.start_draco_server()
        
        self.get_logger().info(f"LiDAR-Draco Bridge started")
        self.get_logger().info(f"Using {self.compression_type} compression (level: {self.compression_level})")
        self.get_logger().info(f"Available transports: {self.transport.get_available_transports()}")
        self.get_logger().info(f"Listening on {self.draco_host}:{self.draco_port}")
        self.get_logger().info(f"Subscribing to: {self.lidar_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic}")
        
    def start_draco_server(self):
        """드라코 서버 시작"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.draco_host, self.draco_port))
            self.server_socket.listen(self.max_clients)
            
            self.running = True
            self.server_thread = threading.Thread(target=self.server_loop)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            self.get_logger().info(f"Draco server started on {self.draco_host}:{self.draco_port}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start Draco server: {e}")
            
    def server_loop(self):
        """서버 루프 - 클라이언트 연결 처리"""
        while self.running:
            try:
                client_socket, address = self.server_socket.accept()
                self.get_logger().info(f"New client connected: {address}")
                
                # 클라이언트를 리스트에 추가
                self.clients.append(client_socket)
                
                # 클라이언트별 처리 스레드 시작
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, address)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Server error: {e}")
                    
    def handle_client(self, client_socket: socket.socket, address):
        """클라이언트 연결 처리"""
        try:
            while self.running:
                # 클라이언트가 살아있는지 확인
                try:
                    client_socket.send(b'ping')
                except:
                    break
                time.sleep(1)
                
        except Exception as e:
            self.get_logger().error(f"Client {address} error: {e}")
        finally:
            # 클라이언트 정리
            if client_socket in self.clients:
                self.clients.remove(client_socket)
            client_socket.close()
            self.get_logger().info(f"Client {address} disconnected")
            
    def point_cloud_callback(self, point_cloud: PointCloud2):
        """라이다 포인트 클라우드 콜백 (압축된 데이터)"""
        self.message_count += 1
        
        # 통계 계산
        num_points = point_cloud.width * point_cloud.height
        data_size = len(point_cloud.data)
        
        # 압축된 데이터를 다시 퍼블리시 (로컬 네트워크용)
        self.publisher.publish(point_cloud)
        
        # 드라코 클라이언트들에게 전송
        self.send_to_draco_clients(point_cloud)
        
        # 로그 출력 (10개 메시지마다)
        if self.message_count % 10 == 0:
            self.get_logger().info(f"Processed {self.message_count} compressed point clouds")
            self.get_logger().info(f"  Points: {num_points}, Compressed size: {data_size} bytes")
            self.get_logger().info(f"  Active clients: {len(self.clients)}")
            
            # 압축률 계산
            original_size = num_points * 16  # 대략적인 원본 크기 (x,y,z,intensity * 4 bytes)
            compression_ratio = original_size / data_size if data_size > 0 else 0
            self.get_logger().info(f"  Compression ratio: {compression_ratio:.2f}:1")
            
    def send_to_draco_clients(self, point_cloud: PointCloud2):
        """모든 드라코 클라이언트에게 압축된 포인트 클라우드 전송"""
        if not self.clients:
            return
            
        # 메시지 헤더 생성
        header = {
            'frame_id': point_cloud.header.frame_id,
            'timestamp': {
                'sec': point_cloud.header.stamp.sec,
                'nanosec': point_cloud.header.stamp.nanosec
            },
            'width': point_cloud.width,
            'height': point_cloud.height,
            'point_step': point_cloud.point_step,
            'row_step': point_cloud.row_step,
            'is_dense': point_cloud.is_dense,
            'compression_type': self.compression_type,
            'compression_level': self.compression_level,
            'message_id': self.message_count,
            'compression_ratio': self.calculate_compression_ratio(point_cloud)
        }
        
        # 헤더 직렬화
        header_json = json.dumps(header).encode('utf-8')
        header_size = len(header_json)
        
        # 데이터 크기
        data_size = len(point_cloud.data)
        
        # 연결이 끊어진 클라이언트 제거
        active_clients = []
        
        for client in self.clients:
            try:
                # 헤더 크기 전송
                client.send(struct.pack('I', header_size))
                # 헤더 전송
                client.send(header_json)
                # 데이터 크기 전송
                client.send(struct.pack('I', data_size))
                # 압축된 포인트 클라우드 데이터 전송
                client.send(point_cloud.data)
                
                active_clients.append(client)
                self.bytes_sent += header_size + data_size + 8  # +8 for size fields
                
            except Exception as e:
                self.get_logger().warn(f"Failed to send to client: {e}")
                try:
                    client.close()
                except:
                    pass
                    
        # 활성 클라이언트 리스트 업데이트
        self.clients = active_clients
        
    def calculate_compression_ratio(self, point_cloud: PointCloud2) -> float:
        """압축률 계산"""
        num_points = point_cloud.width * point_cloud.height
        original_size = num_points * 16  # 대략적인 원본 크기
        compressed_size = len(point_cloud.data)
        return original_size / compressed_size if compressed_size > 0 else 1.0
        
    def print_statistics(self):
        """통계 정보 출력"""
        if self.message_count > 0:
            elapsed_time = time.time() - self.start_time
            rate = self.message_count / elapsed_time
            avg_bytes = self.bytes_sent / self.message_count if self.message_count > 0 else 0
            
            self.get_logger().info("=== Bridge Statistics ===")
            self.get_logger().info(f"Messages sent: {self.message_count}")
            self.get_logger().info(f"Data rate: {rate:.2f} Hz")
            self.get_logger().info(f"Total bytes: {self.bytes_sent}")
            self.get_logger().info(f"Avg message size: {avg_bytes:.0f} bytes")
            self.get_logger().info(f"Active clients: {len(self.clients)}")
            self.get_logger().info(f"Compression: {self.compression_type} (level: {self.compression_level})")
            
            # Transport 통계
            if hasattr(self.subscriber, 'get_statistics'):
                sub_stats = self.subscriber.get_statistics()
                self.get_logger().info(f"Subscriber stats: {sub_stats}")
                
            if hasattr(self.publisher, 'get_statistics'):
                pub_stats = self.publisher.get_statistics()
                self.get_logger().info(f"Publisher stats: {pub_stats}")
            
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.running = False
        
        # 모든 클라이언트 연결 종료
        for client in self.clients:
            try:
                client.close()
            except:
                pass
        self.clients.clear()
        
        # 서버 소켓 종료
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
                
        # Transport 정리
        if hasattr(self, 'subscriber'):
            self.subscriber.shutdown()
        if hasattr(self, 'publisher'):
            self.publisher.shutdown()
                
        super().destroy_node()
