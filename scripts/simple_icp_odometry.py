#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial import KDTree
import math

class SimpleICPOdometry(Node):
    def __init__(self):
        super().__init__('simple_icp_odometry')
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # State
        self.prev_scan = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.get_logger().info('Simple ICP Odometry Node Started')
    
    def scan_to_points(self, scan):
        """LaserScan을 2D 포인트로 변환"""
        points = []
        angle = scan.angle_min
        
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += scan.angle_increment
        
        return np.array(points)
    
    def simple_icp(self, source, target, max_iterations=20):
        """간단한 ICP 구현"""
        if len(source) < 10 or len(target) < 10:
            return 0.0, 0.0, 0.0
        
        # 초기 변환
        dx, dy, dtheta = 0.0, 0.0, 0.0
        
        for _ in range(max_iterations):
            # 변환 적용
            c, s = math.cos(dtheta), math.sin(dtheta)
            transformed = np.array([
                [c * p[0] - s * p[1] + dx, s * p[0] + c * p[1] + dy]
                for p in source
            ])
            
            # 가장 가까운 점 찾기
            tree = KDTree(target)
            distances, indices = tree.query(transformed)
            
            # 평균 거리가 작으면 종료
            if np.mean(distances) < 0.01:
                break
            
            # 중심점 계산
            source_center = np.mean(transformed, axis=0)
            target_center = np.mean(target[indices], axis=0)
            
            # 변환 업데이트
            delta = target_center - source_center
            dx += delta[0] * 0.5
            dy += delta[1] * 0.5
            
            # 회전 추정 (간단한 방법)
            if len(transformed) > 0:
                angle_diff = np.arctan2(delta[1], delta[0])
                dtheta += angle_diff * 0.1
        
        return dx, dy, dtheta
    
    def scan_callback(self, scan):
        """LaserScan 콜백"""
        current_points = self.scan_to_points(scan)
        
        if self.prev_scan is not None:
            prev_points = self.scan_to_points(self.prev_scan)
            
            # ICP로 움직임 추정
            dx, dy, dtheta = self.simple_icp(current_points, prev_points)
            
            # 상태 업데이트
            self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
            self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
            self.theta += dtheta
            
            # TF 발행
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.w = math.cos(self.theta / 2)
            t.transform.rotation.z = math.sin(self.theta / 2)
            
            self.tf_broadcaster.sendTransform(t)
            
            # Odometry 발행
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.w = math.cos(self.theta / 2)
            odom.pose.pose.orientation.z = math.sin(self.theta / 2)
            
            self.odom_pub.publish(odom)
            
            if self.get_clock().now().nanoseconds % 1000000000 < 100000000:
                self.get_logger().info(f'Odom: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')
        
        self.prev_scan = scan

def main(args=None):
    rclpy.init(args=args)
    node = SimpleICPOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

