#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import LaserScan

class JointTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.hareket_1 = 0.0  

       
        self.lidar_subscription = self.create_subscription(
            Float32,
            'mesafe',
            self.lidar_callback,
            10
        )

    def lidar_callback(self, msg):
        
        self.hareket_1 = msg.data
        self.get_logger().info(f'LiDAR Mesafesi alındı: {self.hareket_1} metre')

    def timer_callback(self):
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']
        point = JointTrajectoryPoint()
        point.positions = [self.hareket_1, 0.5, 0.0] 
        point.time_from_start.sec = 10
        point.time_from_start.nanosec = 0
        
        msg.points = [point]

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
