#! /usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import math
import os

from matplotlib import pyplot as plt

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R


class Mapping(Node):
    def __init__(self):
        super().__init__('mapping')
        qos_policy_scan = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1) 
        self.scan_idx = 0
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy_scan)

        qos_policy_odom = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_policy_odom)

        self.subscription_map = self.create_subscription(
            String,
            '/map_file',
            self.map_callback,
            qos_profile=qos_policy_scan
        )        
        self.subscription_finished = self.create_subscription(
            Bool,
            '/has_finished',
            self.finished_callback,
            qos_profile=qos_policy_scan
        )        
        
        self.subscription_twist = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )     

        self.publisher = self.create_publisher(
            String,
            '/map_file',
            qos_profile=qos_policy_scan
        )
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        print("created stuff")

        self.size = 240
        self.room_map = np.zeros([self.size, self.size])
        self.room_map_path = self.room_map.copy()

        print("trying to import map")
        try:
            path = "mymap.csv"
            data = np.genfromtxt(path, delimiter=',')
            self.room_map = data
            print("import successful")
        except:
            print("no file found")
        
        self.position = [0,0,0]
        self.yaw = 0
        self.finished = False
        self.map_drawn = False
        self.robot_positions = []
        self.robot_speeds = []
        self.fig = None
        self.odom_time = 0
        plt.ion()

    def scan_callback(self,msg):
        #early return if rostime is not sufficiently close to odom data rostime
        if abs(self.odom_time - msg.header.stamp.nanosec) > 10000000:
            return
        if self.fig is None:
            self.fig = plt.figure(figsize=(12, 10))
        
        self.fig.clf()
        
        ax1 = self.fig.add_subplot(2, 2, 1)
        if len(self.robot_speeds) > 0:
            ax1.plot(self.robot_speeds, 'b-', linewidth=1.5)
            ax1.set_title("Robot Speed Commands Over Time")
            ax1.set_xlabel("Time (samples)")
            ax1.set_ylabel("Linear speed (m/s)")
            ax1.grid(True)
        else:
            ax1.text(0.5, 0.5, "Waiting for speed data...", ha='center', va='center')


        angles = []
        for i in range(len(msg.ranges)):
            angles.append(msg.angle_min + msg.angle_increment * i)
        
        distances = np.array(msg.ranges)
        
        #filter out invalid measurements
        valid_mask = ~(np.isnan(distances) | np.isinf(distances))
        valid_distances = distances[valid_mask]
        valid_angles = np.array(angles)[valid_mask]
        
        xs_robot = valid_distances * np.cos(valid_angles)
        ys_robot = valid_distances * np.sin(valid_angles)
        
        xs_global = xs_robot * np.cos(self.yaw) - ys_robot * np.sin(self.yaw) + self.position[0]
        ys_global = xs_robot * np.sin(self.yaw) + ys_robot * np.cos(self.yaw) + self.position[1]

        ax2 = self.fig.add_subplot(2, 2, 2)
        ax2.plot(xs_global, ys_global, '.', markersize=1)
        ax2.plot(self.position[0], self.position[1], 'ro', markersize=8, label='Robot')
        ax2.set_title("Global Cartesian Plot (x,y)")
        ax2.axis('equal')
        ax2.legend()
        ax2.grid(True)

        size = 0.12
        map_center = [int(self.size/2), int(self.size/2)]
        
        robot_grid_x = int(self.position[0] / size) + map_center[0]
        robot_grid_y = int(self.position[1] / size) + map_center[1]

        self.robot_positions.append([robot_grid_x, robot_grid_y])
        
        ax3 = self.fig.add_subplot(2, 2, 3)
        ax3.imshow(self.room_map, cmap="gray", origin='lower')
        ax3.set_title("Updated Room Map")

        self.fig.suptitle(f"Lidar Mapping - Robot Position: ({self.position[0]:.2f}, {self.position[1]:.2f})", fontsize=16)

        for i in range(len(xs_global)):
            grid_x = int(xs_global[i] / size) + map_center[0]
            grid_y = int(ys_global[i] / size) + map_center[1]
            
            #coordinates within bounds
            if 0 <= grid_x < self.size and 0 <= grid_y < self.size:
                line_points = bresenham_points([robot_grid_x, robot_grid_y], [grid_x, grid_y])
                
                for point in line_points:
                    if 0 <= point[0] < self.size and 0 <= point[1] < self.size:
                        self.room_map[point[1], point[0]] = 1
                
                self.room_map[grid_y, grid_x] = 2


            
            if 0 <= robot_grid_x < 160 and 0 <= robot_grid_y < 160:
                ax3.plot(robot_grid_x, robot_grid_y, 'ro', markersize=8)

        if not self.finished:
            self.fig.suptitle(f"Lidar Mapping - Robot Position: ({self.position[0]:.2f}, {self.position[1]:.2f})", fontsize=16)
            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.draw()
            plt.pause(0.01)

    def odom_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        r = R.from_quat(quat)
        roll, pitch, self.yaw = r.as_euler('xyz', degrees=False)
        self.odom_time = msg.header.stamp.nanosec
    
    def map_callback(self, msg):
        if self.finished and not self.map_drawn:
            try:
                path = msg.data
                room_map_path = np.genfromtxt(path, delimiter=',')
                print("import successful")
            except:
                print("no file found")
                return

            # overlay robot trajectory
            robot_positions_array = np.array(self.robot_positions)

            print("Drawing map")
            ax4 = self.fig.add_subplot(2, 2, 4)
            ax4.imshow(room_map_path, cmap="gray", origin='lower')
            ax4.plot(robot_positions_array[:,0], robot_positions_array[:,1], 'r-', linewidth=1)
            ax4.set_title("Room Map with path")

            self.fig.suptitle(f"Lidar Mapping - Robot Position: ({self.position[0]:.2f}, {self.position[1]:.2f})", fontsize=16)
            plt.tight_layout(rect=[0, 0.03, 1, 0.95])
            plt.draw()
            plt.show(block=True)  # keep final map visible
            print("Map drawn")
            
    def finished_callback(self, msg):
        self.finished = msg.data
    
    def twist_callback(self, msg):
        self.robot_speeds.append(msg.linear.x)

    def timer_callback(self):
        np.savetxt("mymap.csv", self.room_map, delimiter=",")
        os_path = os.path.abspath("mymap.csv")
        msg = String()
        msg.data = os_path
        self.publisher.publish(msg)
        print("saving and publishing")


def bresenham_points(p0, p1):
    """Bresenham's line algorithm - returns all points between p0 and p1"""
    x0, y0 = int(p0[0]), int(p0[1])
    x1, y1 = int(p1[0]), int(p1[1])
    
    points = []
    
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    
    err = dx - dy
    
    index = 0
    while True and index < 30:
        index += 1
        if (x0 != x1 or y0 != y1) and (x0 != p0[0] or y0 != p0[1]):
            points.append([x0, y0])
        
        if x0 == x1 and y0 == y1:
            break
            
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
            
    return points


def main(args=None):
    rclpy.init(args=args)
    mapping = Mapping()
    try:
        rclpy.spin(mapping)
    except KeyboardInterrupt:
        pass
    finally:
        mapping.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()