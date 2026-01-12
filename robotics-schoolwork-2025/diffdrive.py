"""
Implement the forward kinematics for a differential drive robot.
"""

import numpy as np


def diffdrive(x, y, theta, v_l, v_r, t, l):
    """
    Compute the new pose of a differential drive robot after executing
    a motion command.

    Parameters
    ----------
    x : float
        Current x position in meters
    y : float
        Current y position in meters
    theta : float
        Current orientation in radians
    v_l : float
        Left wheel velocity in m/s
    v_r : float
        Right wheel velocity in m/s
    t : float
        Duration of motion in seconds
    l : float
        Distance between the wheels in meters

    Returns
    -------
    x_n : float
        New x position in meters
    y_n : float
        New y position in meters
    theta_n : float
        New orientation in radians


    """
    speed_avg = (v_r + v_l) / 2.0
    omega = (v_r - v_l) / l # Angular velocity of robot

    theta_n = theta + omega * t
    distance = speed_avg * t
    
    direction_x = np.cos(theta + omega * t / 2)
    direction_y = np.sin(theta + omega * t / 2)

    x_n = x + distance * direction_x
    y_n = y + distance * direction_y

    return x_n, y_n, theta_n


def task_b():
    print("\n\nTask B: Using diffdrive function to perform 3 step motion")
    x, y, theta = 1.5, 2.0, np.pi / 2.0
    l = 0.5

    print(f"Initial pose: ({x:.3f}, {y:.3f}, {theta:.3f})")

    # Movement steps:
    x_new, y_new, theta_new = diffdrive(x, y, theta, 0.3, 0.3, 3.0, l)
    print(f"Step 1 pose: ({x_new:.3f}, {y_new:.3f}, {theta_new:.3f})")

    x_new, y_new, theta_new = diffdrive(x_new, y_new, theta_new, 0.1, -0.1, 1.0, l)
    print(f"Step 2 pose: ({x_new:.3f}, {y_new:.3f}, {theta_new:.3f})")

    x_new, y_new, theta_new = diffdrive(x_new, y_new, theta_new, 0.2, 0.0, 2.0, l)
    print(f"Step 3 pose: ({x_new:.3f}, {y_new:.3f}, {theta_new:.3f})")


if __name__ == "__main__":
    # Simple test case: robot moves straight forward
    print("Test 1: Straight line motion")
    x, y, theta = 0.0, 0.0, 0.0
    v_l, v_r = 1.0, 1.0
    t = 1.0
    l = 0.5

    x_new, y_new, theta_new = diffdrive(x, y, theta, v_l, v_r, t, l)
    print(f"Initial pose: ({x:.3f}, {y:.3f}, {theta:.3f})")
    print(f"Final pose: ({x_new:.3f}, {y_new:.3f}, {theta_new:.3f})")
    print("Expected: (1.000, 0.000, 0.000)")
    print()

    # Test case 2: robot rotates in place
    print("Test 2: Rotation in place")
    x, y, theta = 0.0, 0.0, 0.0
    v_l, v_r = -0.5, 0.5
    t = np.pi  # Rotate for pi seconds
    l = 1.0

    x_new, y_new, theta_new = diffdrive(x, y, theta, v_l, v_r, t, l)
    print(f"Initial pose: ({x:.3f}, {y:.3f}, {theta:.3f})")
    print(f"Final pose: ({x_new:.3f}, {y_new:.3f}, {theta_new:.3f})")
    print(f"Expected: (0.000, 0.000, {np.pi:.3f})")

    task_b()
