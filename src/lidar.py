#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class LiDARNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        
        self.subscription = self.create_subscription(
            LaserScan,
            'xbot/scan',
            self.listener_callback,
            10
        )
        
      
        self.distance_publisher = self.create_publisher(Float32, 'mesafe', 10)

    def listener_callback(self, msg):
        angle_to_measure = 0 
        index = int((angle_to_measure - msg.angle_min) / msg.angle_increment)
        
        if 0 <= index < len(msg.ranges):
            distance = msg.ranges[index]
            self.get_logger().info(f'Açı: {angle_to_measure} derece, Mesafe: {distance} metre')
            
            
            distance_msg = Float32()
            distance_msg.data = distance
            self.distance_publisher.publish(distance_msg)
        else:
            self.get_logger().warn(f'Açı: {angle_to_measure} derece, geçersiz indis')

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LiDARNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

