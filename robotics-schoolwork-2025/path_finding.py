import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        qos_policy_odom = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.callback_odom,
            qos_profile=qos_policy_odom
        )

        self.publisher_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.publisher_bool = self.create_publisher(
            Bool,
            '/has_finished',
            10
        )
        

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        print("created stuff")

        self.waypoints = [
            #(-13.0, -3.0),      # Start position
            (-9.0, -3.0),      # First waypoint
            (-7.5, -3.0),      # Second waypoint
            (-7.5, -1.5),   # Third waypoint
            (-5.5, -1.5),
            (-5.5, -3.0),
            (-0.5, -3.0),
            (-0.5, -5.5),
            (1.5, -5.5),
            (1.5, -0.5),
            (4.5, -0.5),
            (4.5, -3.5),
            (11.0, -3.5) # Final waypoint next to the beer can
        ]
        
        self.current_waypoint = 0
        self.finished = False

        self.linear = Vector3()
        self.angular = Vector3()
    
    def callback_odom(self, msg):
        print("odom callback")
        target_position = self.waypoints[self.current_waypoint]
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        rot = Rotation.from_quat(orientation_list)
        rot_euler = rot.as_euler('xyz', degrees=False)

        yaw = rot_euler[2]
        
        high_speed = 0.6
        slow_speed = 0.1

        self.angular.z = 0.0
        # if negative, rotate right, if positive, rotate left
        if angle_to_point(position, target_position, yaw) > 0.0:
            self.angular.z = slow_speed
            if angle_to_point(position, target_position, yaw) > 0.5:
                self.angular.z = high_speed
        if angle_to_point(position, target_position, yaw) < -0.0:
            self.angular.z = -slow_speed
            if angle_to_point(position, target_position, yaw) < -0.5:
                self.angular.z = -high_speed
        

        self.linear.x = 0.0
        # if within some angle drive forwards
        if abs(angle_to_point(position, target_position, yaw)) < 0.1:
            #slow down as we approch point to lessen overshoot
            if distance_to_point(position, target_position) < 0.8:
                self.linear.x = 0.5
            else:
                self.linear.x = 1.4

        print("linear: " + str(self.linear))
        print("angular: " + str(self.angular))
        
        if distance_to_point(position, target_position) < 0.1:
            
            print("Reached point: " + str(self.current_waypoint + 1) + "!")
            if self.current_waypoint < len(self.waypoints) - 1:
                self.current_waypoint += 1
            else:
                self.finished = True
                self.linear.x = 0.0
                self.angular.z = 0.0

    def timer_callback(self):
        msg_t = Twist()
        msg_t.linear = self.linear
        msg_t.angular = self.angular
        self.publisher_twist.publish(msg_t)

        msg_b = Bool()
        msg_b.data = self.finished
        self.publisher_bool.publish(msg_b)

def distance_to_point(position, point_position):
    dx = point_position[0] - position[0]
    dy = point_position[1] - position[1]
    return math.sqrt(dx*dx + dy*dy)

def angle_to_point(position, point_position, yaw):
    direction = (point_position[0] - position[0], point_position[1] - position[1])
    angle_to = math.atan2(direction[1], direction[0]) - yaw
    return (angle_to + math.pi) % (2 * math.pi) - math.pi

def dot(vec1, vec2):
    return vec1[0] * vec2[0] + vec1[1] * vec2[1]

def cross(vec1, vec2):
    return vec1[0] * vec2[1] - vec1[1] * vec2[0]

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    try:
        rclpy.spin(waypoint_follower)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()