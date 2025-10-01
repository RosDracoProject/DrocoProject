#!/usr/bin/env python3

import socket
import json
import struct
import time
import numpy as np
from typing import Optional, Callable, Dict, Any
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from point_cloud_transport_py import PointCloudTransport


class ImprovedDracoClient(Node):
    """
    개선된 드라코 클라이언트: point_cloud_transport_py를 사용한 효율적인 데이터 수신
    """
    
    def __init__(self, host: str = 'localhost', port: int = 8080):
        super().__init__('improved_draco_client')
        
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.connected = False
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        
        # 통계
        self.message_count = 0
        self.bytes_received = 0
        self.start_time = time.time()
        self.compression_ratios = []
        
        # 콜백 함수
        self.point_cloud_callback: Optional[Callable] = None
        
        # PointCloudTransport 초기화
        self.transport = PointCloudTransport(self)
        
        # QoS 프로파일 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 압축된 포인트 클라우드 구독자 생성 (로컬 네트워크용)
        self.subscriber = self.transport.subscribe(
            "/lidar/compressed",
            self.local_point_cloud_callback,
            qos_profile=qos_profile,
            transport='draco'  # 또는 'zlib'
        )
        
        self.get_logger().info(f"Improved Draco Client started")
        self.get_logger().info(f"Available transports: {self.transport.get_available_transports()}")
        
    def connect(self) -> bool:
        """드라코 서버에 연결"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            self.get_logger().info(f"Connected to Draco server at {self.host}:{self.port}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Draco server: {e}")
            return False
            
    def disconnect(self):
        """서버 연결 해제"""
        self.running = False
        self.connected = False
        
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
            
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
            
        self.get_logger().info("Disconnected from Draco server")
        
    def start_receiving(self):
        """데이터 수신 시작"""
        if not self.connected:
            self.get_logger().error("Not connected to server")
            return False
            
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        self.get_logger().info("Started receiving data from Draco server")
        return True
        
    def _receive_loop(self):
        """데이터 수신 루프"""
        while self.running and self.connected:
            try:
                # 헤더 크기 수신
                header_size_data = self._receive_exact(4)
                if not header_size_data:
                    break
                    
                header_size = struct.unpack('I', header_size_data)[0]
                
                # 헤더 수신
                header_data = self._receive_exact(header_size)
                if not header_data:
                    break
                    
                header = json.loads(header_data.decode('utf-8'))
                
                # 데이터 크기 수신
                data_size_data = self._receive_exact(4)
                if not data_size_data:
                    break
                    
                data_size = struct.unpack('I', data_size_data)[0]
                
                # 포인트 클라우드 데이터 수신
                point_cloud_data = self._receive_exact(data_size)
                if not point_cloud_data:
                    break
                    
                # 통계 업데이트
                self.message_count += 1
                self.bytes_received += header_size + data_size + 8
                
                # 압축률 기록
                if 'compression_ratio' in header:
                    self.compression_ratios.append(header['compression_ratio'])
                
                # 포인트 클라우드 처리
                self._process_point_cloud(header, point_cloud_data)
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Receive error: {e}")
                break
                
    def _receive_exact(self, size: int) -> Optional[bytes]:
        """정확한 크기만큼 데이터 수신"""
        data = b''
        while len(data) < size:
            try:
                chunk = self.socket.recv(size - len(data))
                if not chunk:
                    return None
                data += chunk
            except Exception as e:
                self.get_logger().error(f"Receive error: {e}")
                return None
        return data
        
    def _process_point_cloud(self, header: dict, data: bytes):
        """포인트 클라우드 데이터 처리"""
        # 포인트 클라우드 정보 출력
        if self.message_count % 10 == 0:
            self.get_logger().info(f"Received point cloud #{self.message_count}:")
            self.get_logger().info(f"  Frame ID: {header.get('frame_id', 'unknown')}")
            self.get_logger().info(f"  Size: {header.get('width', 0)} x {header.get('height', 0)}")
            self.get_logger().info(f"  Points: {header.get('width', 0) * header.get('height', 0)}")
            self.get_logger().info(f"  Data size: {len(data)} bytes")
            self.get_logger().info(f"  Compression: {header.get('compression_type', 'unknown')}")
            if 'compression_ratio' in header:
                self.get_logger().info(f"  Compression ratio: {header['compression_ratio']:.2f}:1")
            
        # 콜백 함수 호출
        if self.point_cloud_callback:
            try:
                self.point_cloud_callback(header, data)
            except Exception as e:
                self.get_logger().error(f"Callback error: {e}")
                
    def local_point_cloud_callback(self, point_cloud: PointCloud2):
        """로컬 네트워크에서 수신한 압축된 포인트 클라우드 콜백"""
        self.get_logger().info(f"Received local compressed point cloud:")
        self.get_logger().info(f"  Frame ID: {point_cloud.header.frame_id}")
        self.get_logger().info(f"  Size: {point_cloud.width} x {point_cloud.height}")
        self.get_logger().info(f"  Data size: {len(point_cloud.data)} bytes")
        
        # 로컬 처리 로직
        self.process_local_point_cloud(point_cloud)
        
    def process_local_point_cloud(self, point_cloud: PointCloud2):
        """로컬 포인트 클라우드 처리"""
        # 여기서 압축된 포인트 클라우드를 처리할 수 있습니다
        # 예: 압축 해제, 시각화, 분석 등
        pass
        
    def set_point_cloud_callback(self, callback: Callable[[dict, bytes], None]):
        """포인트 클라우드 콜백 함수 설정"""
        self.point_cloud_callback = callback
        
    def get_statistics(self) -> dict:
        """통계 정보 반환"""
        elapsed_time = time.time() - self.start_time
        rate = self.message_count / elapsed_time if elapsed_time > 0 else 0
        avg_bytes = self.bytes_received / self.message_count if self.message_count > 0 else 0
        avg_compression_ratio = np.mean(self.compression_ratios) if self.compression_ratios else 0
        
        return {
            'message_count': self.message_count,
            'bytes_received': self.bytes_received,
            'data_rate': rate,
            'avg_message_size': avg_bytes,
            'avg_compression_ratio': avg_compression_ratio,
            'elapsed_time': elapsed_time
        }
        
    def print_statistics(self):
        """통계 정보 출력"""
        stats = self.get_statistics()
        self.get_logger().info("=== Improved Draco Client Statistics ===")
        self.get_logger().info(f"Messages received: {stats['message_count']}")
        self.get_logger().info(f"Data rate: {stats['data_rate']:.2f} Hz")
        self.get_logger().info(f"Total bytes: {stats['bytes_received']}")
        self.get_logger().info(f"Avg message size: {stats['avg_message_size']:.0f} bytes")
        self.get_logger().info(f"Avg compression ratio: {stats['avg_compression_ratio']:.2f}:1")
        self.get_logger().info(f"Elapsed time: {stats['elapsed_time']:.1f}s")
        
        # Transport 통계
        if hasattr(self.subscriber, 'get_statistics'):
            sub_stats = self.subscriber.get_statistics()
            self.get_logger().info(f"Local subscriber stats: {sub_stats}")
            
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.disconnect()
        
        # Transport 정리
        if hasattr(self, 'subscriber'):
            self.subscriber.shutdown()
            
        super().destroy_node()


def main():
    """개선된 드라코 클라이언트 테스트"""
    rclpy.init()
    
    try:
        client = ImprovedDracoClient('localhost', 8080)
        
        def point_cloud_callback(header: dict, data: bytes):
            """포인트 클라우드 콜백 예제"""
            client.get_logger().info(f"Received point cloud: {header.get('frame_id', 'unknown')}")
            
        # 콜백 설정
        client.set_point_cloud_callback(point_cloud_callback)
        
        # 서버 연결
        if not client.connect():
            return
            
        # 데이터 수신 시작
        if not client.start_receiving():
            return
            
        client.get_logger().info("Improved Draco client running. Press Ctrl+C to stop.")
        
        # 주기적 통계 출력
        timer = client.create_timer(5.0, client.print_statistics)
        
        # ROS2 스핀
        rclpy.spin(client)
        
    except KeyboardInterrupt:
        client.get_logger().info("Stopping improved Draco client...")
    finally:
        if 'client' in locals():
            client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
