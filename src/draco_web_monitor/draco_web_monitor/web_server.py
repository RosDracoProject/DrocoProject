#!/usr/bin/env python3

import os
import sys
import time
import json
import threading
from flask import Flask, render_template, jsonify, request
from collections import deque
import psutil
import netifaces

class WebMonitor:
    def __init__(self, port=5000):
        # 템플릿 폴더의 절대 경로 설정
        template_dir = os.path.join(os.path.dirname(__file__), 'templates')
        self.app = Flask(__name__, 
                        template_folder=template_dir,
                        static_folder='static')
        self.port = port
        
        # 메트릭 데이터 저장 (클라이언트에서 측정 가능한 것들만)
        self.metrics = {
            'compression_ratio': deque(maxlen=100),
            'decompression_time': deque(maxlen=100),
            'network_throughput': deque(maxlen=100),
            'cpu_usage': deque(maxlen=100),
            'memory_usage': deque(maxlen=100),
            'timestamp': deque(maxlen=100),
            'compressed_size': deque(maxlen=100),
            'decompressed_size': deque(maxlen=100)
        }
        
        # 네트워크 인터페이스 정보
        self.network_interfaces = self._get_network_interfaces()
        
        # 라우트 설정
        self._setup_routes()
        
        # 더미 데이터 생성 (테스트용)
        self._generate_dummy_data()
    
    def _get_network_interfaces(self):
        """네트워크 인터페이스 정보 가져오기"""
        interfaces = {}
        try:
            for interface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addrs:
                    ip = addrs[netifaces.AF_INET][0]['addr']
                    if not ip.startswith('127.'):
                        interfaces[interface] = ip
        except Exception as e:
            print(f"네트워크 인터페이스 정보 가져오기 실패: {e}")
            interfaces = {'eth0': '192.168.1.100', 'wlan0': '192.168.1.101'}
        return interfaces
    
    def _generate_dummy_data(self):
        """더미 데이터 생성 (테스트용)"""
        import random
        current_time = time.time()
        
        for i in range(20):
            compressed_size = random.uniform(50000, 150000)
            decompressed_size = compressed_size * (18.0 + random.uniform(-1, 1))
            
            self.metrics['compression_ratio'].append(18.0 + random.uniform(-1, 1))
            self.metrics['decompression_time'].append(random.uniform(3.0, 8.0))
            self.metrics['network_throughput'].append(random.uniform(800, 1500))
            self.metrics['cpu_usage'].append(random.uniform(20, 80))
            self.metrics['memory_usage'].append(random.uniform(30, 70))
            self.metrics['timestamp'].append(current_time - (20-i) * 2)
            self.metrics['compressed_size'].append(compressed_size)
            self.metrics['decompressed_size'].append(decompressed_size)
    
    def _setup_routes(self):
        """Flask 라우트 설정"""
        
        @self.app.route('/')
        def index():
            # 템플릿 파일을 직접 읽어서 반환
            try:
                template_path = os.path.join(os.path.dirname(__file__), 'templates', 'index.html')
                with open(template_path, 'r', encoding='utf-8') as f:
                    return f.read()
            except FileNotFoundError:
                return f"<h1>Template not found</h1><p>Template path: {template_path}</p>", 404
        
        @self.app.route('/test')
        def test():
            return render_template('test.html')
        
        @self.app.route('/api/metrics')
        def get_metrics():
            """현재 메트릭 데이터 반환"""
            return jsonify({
                'compression_ratio': list(self.metrics['compression_ratio']),
                'decompression_time': list(self.metrics['decompression_time']),
                'network_throughput': list(self.metrics['network_throughput']),
                'cpu_usage': list(self.metrics['cpu_usage']),
                'memory_usage': list(self.metrics['memory_usage']),
                'timestamp': list(self.metrics['timestamp']),
                'compressed_size': list(self.metrics['compressed_size']),
                'decompressed_size': list(self.metrics['decompressed_size']),
                'network_interfaces': self.network_interfaces
            })
        
        @self.app.route('/api/compression_data', methods=['POST'])
        def update_compression_data():
            """압축 성능 데이터 업데이트"""
            data = request.json
            current_time = time.time()
            
            if 'compression_ratio' in data:
                self.metrics['compression_ratio'].append(data['compression_ratio'])
            if 'decompression_time' in data:
                self.metrics['decompression_time'].append(data['decompression_time'])
            if 'network_throughput' in data:
                self.metrics['network_throughput'].append(data['network_throughput'])
            if 'compressed_size' in data:
                self.metrics['compressed_size'].append(data['compressed_size'])
            if 'decompressed_size' in data:
                self.metrics['decompressed_size'].append(data['decompressed_size'])
            
            return jsonify({'status': 'success'})
        
        @self.app.route('/api/system_info')
        def get_system_info():
            """시스템 정보 반환"""
            return jsonify({
                'cpu_percent': psutil.cpu_percent(),
                'memory_percent': psutil.virtual_memory().percent,
                'network_interfaces': self.network_interfaces
            })
    
    def start(self):
        """웹 서버 시작"""
        print(f"웹 모니터링 서버가 http://localhost:{self.port} 에서 실행됩니다.")
        print(f"템플릿 폴더: {self.app.template_folder}")
        self.app.run(host='0.0.0.0', port=self.port, debug=False, threaded=True)

if __name__ == '__main__':
    monitor = WebMonitor()
    monitor.start()
