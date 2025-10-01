#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
import sensor_msgs_py.point_cloud2 as pc2

# KISS-SLAM imports
from kiss_slam.pipeline import KissSLAM
from kiss_slam.config import KissSLAMConfig

class KissSLAMNode(Node):
    def __init__(self):
        super().__init__('kiss_slam_node')
        
        # KISS-SLAM 설정
        config = KissSLAMConfig()
        config.odometry.preprocessing.max_range = 100.0
        config.odometry.preprocessing.min_range = 0.5
        config.odometry.mapping.voxel_size = 0.5
        
        self.slam = KissSLAM(config)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/kiss_slam/odometry', 10)
        self.path_pub = self.create_publisher(Path, '/kiss_slam/path', 10)
        self.map_pub = self.create_publisher(PointCloud2, '/kiss_slam/map', 10)
        self.frame_pub = self.create_publisher(PointCloud2, '/kiss_slam/frame', 10)
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
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        self.get_logger().info('KISS-SLAM ROS2 Node Started!')
        self.get_logger().info('Subscribing to: /sensing/lidar/top/pointcloud_raw_ex')
        self.get_logger().info('Publishing map to: /kiss_slam/map')
    
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
        
        # KISS-SLAM 실행
        try:
            # timestamps: 각 포인트의 타임스탬프
            timestamps = np.zeros(len(points))
            
            # SLAM 처리
            self.slam.process_scan(points, timestamps)
            
            # Pose 가져오기 (가장 최근 pose)
            poses = self.slam.poses()
            if len(poses) == 0:
                return
            
            pose = poses[-1]  # 가장 최근 pose
            
            # Keyposes (맵 포인트) 가져오기
            keyposes = self.slam.get_keyposes()
            local_map = np.vstack([kp.points for kp in keyposes]) if len(keyposes) > 0 else np.array([])
            
            # Pose를 TF로 변환
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = pose[0, 3]
            t.transform.translation.y = pose[1, 3]
            t.transform.translation.z = pose[2, 3]
            
            # 회전 행렬 → 쿼터니언
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
            
            # Path 업데이트
            pose_stamped = PoseStamped()
            pose_stamped.header = t.header
            pose_stamped.pose = odom.pose.pose
            self.path.poses.append(pose_stamped)
            self.path.header.stamp = msg.header.stamp
            self.path_pub.publish(self.path)
            
            # Local map 발행
            if len(local_map) > 0:
                map_msg = self.numpy_to_pointcloud2(local_map, msg.header.stamp, 'odom')
                self.map_pub.publish(map_msg)
            
            # 현재 프레임 발행
            frame_msg = self.numpy_to_pointcloud2(points, msg.header.stamp, 'base_link')
            self.frame_pub.publish(frame_msg)
            
            # 주기적으로 로그 출력
            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f'Frame {self.frame_count}: Pose=({pose[0,3]:.2f}, {pose[1,3]:.2f}, {pose[2,3]:.2f}), '
                    f'Points={len(points)}, Map={len(local_map)}'
                )
        
        except Exception as e:
            self.get_logger().error(f'KISS-SLAM error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = KissSLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

