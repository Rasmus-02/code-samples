import math
import numpy as np
import matplotlib.pyplot as plt

samples = 5000
# Initial pose
x_t = np.array([2.0, 4.0, 0.0])
u_t = np.array([math.pi/2, 1, 0.0])
alpha = np.array([0.1, 0.1, 0.01, 0.01])
all_poses = np.zeros((samples, 3))



def sample_motion_model_odometry ( x_t , u_t , alpha ) :
    """
    Sample from odometry - based motion model .

    Args :
    x_t: Current pose [x, y, theta ] ( numpy array )
    u_t: Odometry reading [ delta_rot1 , delta_trans , delta_rot2 ]
    alpha : Noise parameters [alpha1 , alpha2 , alpha3 , alpha4 ]

    Returns :
    x_t1 : New pose [x, y, theta ] sampled from motion model
    """
    delta_rot1, delta_trans, delta_rot2 = u_t
    x, y, theta = x_t

    sigma_rot1 = math.sqrt(alpha[0] * delta_rot1**2 + alpha[1] * delta_trans**2)
    sigma_rot2 = math.sqrt(alpha[0] * delta_rot2**2 + alpha[1] * delta_trans**2)
    sigma_trans = math.sqrt(alpha[2] * delta_trans**2 + alpha[3] * (delta_rot1**2 + delta_rot2**2))
    
    delta_rot1_hat = delta_rot1 + np.random.normal(0, sigma_rot1)
    delta_rot2_hat = delta_rot2 + np.random.normal(0, sigma_rot2)
    delta_trans_hat = delta_trans + np.random.normal(0, sigma_trans)

    new_x = x + delta_trans_hat * math.cos(theta + delta_rot1_hat)
    new_y = y + delta_trans_hat * math.sin(theta + delta_rot1_hat)
    new_theta = theta + delta_rot1_hat + delta_rot2_hat

    x_t1 = np.array([new_x, new_y, new_theta])
    return x_t1



if __name__ == "__main__":
    for i in range(samples):
        x_t1 = sample_motion_model_odometry(x_t, u_t, alpha)
        print(f"The new pose for sample #{i}: X = {x_t1[0]}, Y = {x_t1[1]}, Theta = {x_t1[2]}")
        all_poses[i, :] = x_t1

        x = all_poses[:, 0]
        y = all_poses[:, 1]

    # Calculate mean and covariance of position distribution
    position = all_poses[:, 0:2]
    mean_position = np.mean(position, axis=0)
    covariance = np.cov(position, rowvar=False)
    print(f"Mean position: X = {mean_position[0]}, Y = {mean_position[1]}")
    print(f"Covariance matrix:\n{covariance}")

    # Calculate theoretical position
    theoretical_x = x_t[0] + u_t[1] * np.cos(x_t[2] + u_t[0])
    theoretical_y = x_t[1] + u_t[1] * np.sin(x_t[2] + u_t[0])

    # Plot samples
    plt.scatter(x, y, s=1, alpha=1)
    plt.scatter(x_t[0], x_t[1], color='green')
    plt.scatter(theoretical_x, theoretical_y, color='red')
    #plt.xlim(1.5, 2.5) 
    #plt.ylim(3, 5)
    plt.show()
