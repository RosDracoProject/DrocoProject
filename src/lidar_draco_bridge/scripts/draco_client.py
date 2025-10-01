#!/usr/bin/env python3

import socket
import json
import struct
import time
import numpy as np
from typing import Optional, Callable, Any
import threading


class DracoClient:
    """
    드라코 서버에 연결하여 라이다 데이터를 수신하는 클라이언트
    """
    
    def __init__(self, host: str = 'localhost', port: int = 8080):
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
        
        # 콜백 함수
        self.point_cloud_callback: Optional[Callable] = None
        
    def connect(self) -> bool:
        """드라코 서버에 연결"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to Draco server at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to Draco server: {e}")
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
            
        print("Disconnected from Draco server")
        
    def start_receiving(self):
        """데이터 수신 시작"""
        if not self.connected:
            print("Not connected to server")
            return False
            
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        print("Started receiving data from Draco server")
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
                
                # 포인트 클라우드 처리
                self._process_point_cloud(header, point_cloud_data)
                
            except Exception as e:
                if self.running:
                    print(f"Receive error: {e}")
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
                print(f"Receive error: {e}")
                return None
        return data
        
    def _process_point_cloud(self, header: dict, data: bytes):
        """포인트 클라우드 데이터 처리"""
        # 포인트 클라우드 정보 출력
        if self.message_count % 10 == 0:
            print(f"Received point cloud #{self.message_count}:")
            print(f"  Frame ID: {header.get('frame_id', 'unknown')}")
            print(f"  Size: {header.get('width', 0)} x {header.get('height', 0)}")
            print(f"  Points: {header.get('width', 0) * header.get('height', 0)}")
            print(f"  Data size: {len(data)} bytes")
            print(f"  Compression: {header.get('compression_enabled', False)}")
            
        # 콜백 함수 호출
        if self.point_cloud_callback:
            try:
                self.point_cloud_callback(header, data)
            except Exception as e:
                print(f"Callback error: {e}")
                
    def set_point_cloud_callback(self, callback: Callable[[dict, bytes], None]):
        """포인트 클라우드 콜백 함수 설정"""
        self.point_cloud_callback = callback
        
    def get_statistics(self) -> dict:
        """통계 정보 반환"""
        elapsed_time = time.time() - self.start_time
        rate = self.message_count / elapsed_time if elapsed_time > 0 else 0
        avg_bytes = self.bytes_received / self.message_count if self.message_count > 0 else 0
        
        return {
            'message_count': self.message_count,
            'bytes_received': self.bytes_received,
            'data_rate': rate,
            'avg_message_size': avg_bytes,
            'elapsed_time': elapsed_time
        }
        
    def print_statistics(self):
        """통계 정보 출력"""
        stats = self.get_statistics()
        print("=== Draco Client Statistics ===")
        print(f"Messages received: {stats['message_count']}")
        print(f"Data rate: {stats['data_rate']:.2f} Hz")
        print(f"Total bytes: {stats['bytes_received']}")
        print(f"Avg message size: {stats['avg_message_size']:.0f} bytes")
        print(f"Elapsed time: {stats['elapsed_time']:.1f}s")


def main():
    """드라코 클라이언트 테스트"""
    client = DracoClient('localhost', 8080)
    
    def point_cloud_callback(header: dict, data: bytes):
        """포인트 클라우드 콜백 예제"""
        print(f"Received point cloud: {header.get('frame_id', 'unknown')}")
        
    # 콜백 설정
    client.set_point_cloud_callback(point_cloud_callback)
    
    try:
        # 서버 연결
        if not client.connect():
            return
            
        # 데이터 수신 시작
        if not client.start_receiving():
            return
            
        print("Draco client running. Press Ctrl+C to stop.")
        
        # 주기적 통계 출력
        while True:
            time.sleep(5)
            client.print_statistics()
            
    except KeyboardInterrupt:
        print("\nStopping Draco client...")
    finally:
        client.disconnect()


if __name__ == '__main__':
    main()
