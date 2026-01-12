import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LidarPlotter(Node):
    def __init__(self):
        super().__init__('lidar_plotter')

        #qos_profile = 10
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
)

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.feature_scan_subscription = self.create_subscription(
            LaserScan,
            '/feature_scan',
            self.feature_scan_callback,
            qos_profile
        )
        self.front_scan_subscription = self.create_subscription(
            LaserScan,
            '/front_scan',
            self.front_scan_callback,
            qos_profile
        )


        # subscribe to both new created topics (/feature_scan and /front_scan)

        self.scan_data = None
        self.feature_scan_data = None
        self.front_scan_data = None

        # Matplotlib setup
        # Matplotlib setup
        self.fig, (self.ax_polar, self.ax_cart) = plt.subplots(1, 2, figsize=(12, 6))
        plt.ion()
        plt.show()


    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, angle_increment):
        """Convert LaserScan polar data to Cartesian coordinates."""
        # please complete this function to return required values
        num_points = len(ranges)
        angles = np.arange(angle_min, angle_min + num_points * angle_increment, angle_increment)
        # remember to clip invalid values (inf, nan), if needed
        # Filter out invalid values (inf, nan, out of range)
        valid_mask = np.isfinite(ranges) & (ranges >= 0) & (ranges <= 100)  # Assuming reasonable range
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Convert to Cartesian coordinates
        x_cart = valid_ranges * np.cos(valid_angles)
        y_cart = valid_ranges * np.sin(valid_angles)



        return angles, ranges, x_cart, y_cart

    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg
        self.update_plot()

    # write two similar callback functions for the other two topics

    def feature_scan_callback(self, msg: LaserScan):
        self.feature_scan_data = msg
        self.update_plot()

    def front_scan_callback(self, msg: LaserScan):
        self.front_scan_data = msg
        self.update_plot()

    def update_plot(self):
        # Only update if we have scan data
        if self.scan_data is None:
            return
            
        # Clear previous plots
        self.ax_polar.clear()
        self.ax_cart.clear()
        
        # Plot original scan data
        if self.scan_data is not None:
            angles, ranges, x_cart, y_cart = self.polar_to_cartesian_coordinate(
                np.array(self.scan_data.ranges),
                self.scan_data.angle_min,
                self.scan_data.angle_max,
                self.scan_data.angle_increment
            )
            self.ax_polar.scatter(angles, ranges, c='black', s=2, alpha=0.7, label='Original Scan')
            self.ax_cart.scatter(x_cart, y_cart, c='black', s=2, alpha=0.7, label='Original Scan')
        
        # Plot feature scan data
        if self.feature_scan_data is not None:
            angles, ranges, x_cart, y_cart = self.polar_to_cartesian_coordinate(
                np.array(self.feature_scan_data.ranges),
                self.feature_scan_data.angle_min,
                self.feature_scan_data.angle_max,
                self.feature_scan_data.angle_increment
            )
            # Filter out zeros (which represent filtered points)
            non_zero_mask = ranges > 0
            if np.any(non_zero_mask):
                self.ax_polar.scatter(angles[non_zero_mask], ranges[non_zero_mask], 
                                    c='red', s=20, marker='x', label='Feature Scan')
                self.ax_cart.scatter(x_cart[non_zero_mask], y_cart[non_zero_mask], 
                                   c='red', s=20, marker='x', label='Feature Scan')
        
        # Plot front scan data
        if self.front_scan_data is not None:
            angles, ranges, x_cart, y_cart = self.polar_to_cartesian_coordinate(
                np.array(self.front_scan_data.ranges),
                self.front_scan_data.angle_min,
                self.front_scan_data.angle_max,
                self.front_scan_data.angle_increment
            )
            # Filter out zeros (which represent filtered points)
            non_zero_mask = ranges > 0
            if np.any(non_zero_mask):
                self.ax_polar.scatter(angles[non_zero_mask], ranges[non_zero_mask], 
                                    c='blue', s=10, alpha=0.8, label='Front Scan')
                self.ax_cart.scatter(x_cart[non_zero_mask], y_cart[non_zero_mask], 
                                   c='blue', s=10, alpha=0.8, label='Front Scan')
        
        # Configure polar plot
        self.ax_polar.set_title('Lidar Data - Polar Coordinates')
        self.ax_polar.set_xlabel('Angle (radians)')
        self.ax_polar.set_ylabel('Range (meters)')
        self.ax_polar.legend()
        self.ax_polar.grid(True)
        
        # Configure Cartesian plot
        self.ax_cart.set_title('Lidar Data - Cartesian Coordinates')
        self.ax_cart.set_xlabel('X (meters)')
        self.ax_cart.set_ylabel('Y (meters)')
        self.ax_cart.legend()
        self.ax_cart.grid(True)
        self.ax_cart.set_aspect('equal', adjustable='box')
        
        # Adjust layout
        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
