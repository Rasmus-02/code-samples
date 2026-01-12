#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

from matplotlib import pyplot as plt

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped



class FeatureExtracter(Node):
    def __init__(self):
        super().__init__('feature_extracter')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 
        self.scan_idx = 0
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)
        
        self.publisher_ = self.create_publisher(LaserScan, '/feature_scan', 10)
        # create a new publisher for /front_scan
        self.publisher_front_ = self.create_publisher(LaserScan, '/front_scan', 10)

        self.publisher_original_ = self.create_publisher(LaserScan, '/original_scan', 10)
        
        # Add transform information to rviz2
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'  # parent
        static_transform.child_frame_id = 'base_scan'   # child
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(static_transform)

    def scan_callback(self,msg):
        ranges = []
        for r in msg.ranges:
            if r > 2.5 or r < 1:   # Feature extraction code here
                ranges.append(0.0)
            else:
                ranges.append(r)
        ################################## 

        ranges_front = []
        increment = msg.angle_increment
        current_angle = 0.0
        for r in msg.ranges:
            
            if current_angle < 2.14 or current_angle > 4.14:   # Feature extraction code here
                ranges_front.append(0.0)
            else:
                ranges_front.append(r)
            
            current_angle += increment

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max 
        scan.ranges = ranges
        self.publisher_.publish(scan)

        scan_front = LaserScan()
        scan_front.header.stamp = msg.header.stamp
        scan_front.header.frame_id = msg.header.frame_id
        scan_front.angle_min = msg.angle_min
        scan_front.angle_max = msg.angle_max
        scan_front.angle_increment = msg.angle_increment
        scan_front.time_increment = msg.time_increment
        scan_front.range_min = msg.range_min
        scan_front.range_max = msg.range_max 
        scan_front.ranges = ranges_front
        self.publisher_front_.publish(scan_front)


        scan_original = LaserScan()
        scan_original.header.stamp = msg.header.stamp
        scan_original.header.frame_id = msg.header.frame_id
        scan_original.angle_min = msg.angle_min
        scan_original.angle_max = msg.angle_max
        scan_original.angle_increment = msg.angle_increment
        scan_original.time_increment = msg.time_increment
        scan_original.range_min = msg.range_min
        scan_original.range_max = msg.range_max 
        scan_original.ranges = msg.ranges
        self.publisher_original_.publish(scan_original)

        self.scan_idx += 1
        print("Publish feature scan message idx", self.scan_idx)



def main(args=None):
    rclpy.init(args=args)
    feature_extracter = FeatureExtracter()
    rclpy.spin(feature_extracter) 
    feature_extracter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
