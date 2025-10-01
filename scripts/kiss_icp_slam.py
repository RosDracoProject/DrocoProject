#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from kiss_icp.kiss_icp import KissICP
from kiss_icp.config import KISSConfig
import sensor_msgs_py.point_cloud2 as pc2

class KissICPNode(Node):
    def __init__(self):
        super().__init__('kiss_icp_slam')
        
        # KISS-ICP 설정
        config = KISSConfig()
        config.data.max_range = 100.0
        config.data.min_range = 0.5
        config.mapping.voxel_size = 0.5  # 중요: voxel 크기 설정
        self.odometry = KissICP(config)
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/kiss/odometry', 10)
        self.map_pub = self.create_publisher(PointCloud2, '/kiss/local_map', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud_raw_ex',
            self.pointcloud_callback,
            10
        )
        
        # State
        self.frame_count = 0
        
        self.get_logger().info('KISS-ICP SLAM Node Started!')
        self.get_logger().info('Subscribing to: /sensing/lidar/top/pointcloud_raw_ex')
        self.get_logger().info('Publishing odometry to: /kiss/odometry')
        self.get_logger().info('Publishing map to: /kiss/local_map')
    
    def pointcloud_to_numpy(self, cloud_msg):
        """PointCloud2 메시지를 numpy 배열로 변환"""
        points = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        if len(points) == 0:
            return np.array([])
        
        return np.array(points, dtype=np.float64)
    
    def numpy_to_pointcloud2(self, points, stamp, frame_id):
        """numpy 배열을 PointCloud2 메시지로 변환"""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        
        if len(points) == 0:
            return msg
        
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        
        return msg
    
    def pointcloud_callback(self, msg):
        """PointCloud2 콜백"""
        self.frame_count += 1
        
        # PointCloud2 → numpy
        points = self.pointcloud_to_numpy(msg)
        
        if len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return
        
        # KISS-ICP 실행
        try:
            # timestamps: 각 포인트의 타임스탬프 (모두 같은 값으로 설정)
            timestamps = np.zeros(len(points))  # 또는 실제 타임스탬프 배열
            pose = self.odometry.register_frame(points, timestamps)
            local_map = self.odometry.local_map()
            
            # Pose를 TF로 변환
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = pose[0, 3]
            t.transform.translation.y = pose[1, 3]
            t.transform.translation.z = pose[2, 3]
            
            # 회전 행렬 → 쿼터니언
            from scipy.spatial.transform import Rotation as R
            rot = R.from_matrix(pose[:3, :3])
            quat = rot.as_quat()
            
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t)
            
            # Odometry 메시지 발행
            odom = Odometry()
            odom.header = t.header
            odom.child_frame_id = t.child_frame_id
            odom.pose.pose.position.x = pose[0, 3]
            odom.pose.pose.position.y = pose[1, 3]
            odom.pose.pose.position.z = pose[2, 3]
            odom.pose.pose.orientation = t.transform.rotation
            
            self.odom_pub.publish(odom)
            
            # Local map 발행
            if len(local_map) > 0:
                map_msg = self.numpy_to_pointcloud2(local_map, msg.header.stamp, 'odom')
                self.map_pub.publish(map_msg)
            
            # 주기적으로 로그 출력
            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f'Frame {self.frame_count}: Pose=({pose[0,3]:.2f}, {pose[1,3]:.2f}, {pose[2,3]:.2f}), '
                    f'Points={len(points)}, Map={len(local_map)}'
                )
        
        except Exception as e:
            self.get_logger().error(f'KISS-ICP error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = KissICPNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

