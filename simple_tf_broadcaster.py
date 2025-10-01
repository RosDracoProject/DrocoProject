#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class SimpleTfBroadcaster(Node):
    def __init__(self):
        super().__init__('simple_tf_broadcaster')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing transforms
        self.timer = self.create_timer(0.05, self.publish_transform)  # 20Hz
        
        self.start_time = time.time()
        
        self.get_logger().info('Simple TF Broadcaster started')
    
    def publish_transform(self):
        transform = TransformStamped()
        
        # Set header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'rslidar_top'
        
        # Calculate circular motion
        current_time = time.time() - self.start_time
        angle = 0.3 * current_time  # Slow rotation
        radius = 3.0
        
        # Circular path
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 0.0
        
        # Set translation
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        # Set rotation (robot always faces forward)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(angle / 2.0)
        transform.transform.rotation.w = math.cos(angle / 2.0)
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)
        
        # Log position every 3 seconds
        if int(current_time) % 3 == 0 and current_time - int(current_time) < 0.1:
            self.get_logger().info(f'Robot position: ({x:.2f}, {y:.2f}, {z:.2f}), angle: {angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTfBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
