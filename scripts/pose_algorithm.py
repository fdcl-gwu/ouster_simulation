import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import numpy as np
import os
import pyvista as pv

num_samples = 100
# Constants
theta_B_C = [np.pi/4, 7*np.pi/4,]
phi_B_C = [-np.pi/8, np.pi/3]
r_C = [0,12]

theta_B_F = [-np.pi/4, np.pi/4,]
phi_B_F = [-np.pi/2, np.pi/2]
r_F = [0,10]

# # Ranges for Cartesian (not used)
# F_range_x = [1, 10]
# F_range_y = [-4.5, 4.5]
# F_range_z = [-1, 4.4]
# C_range_x = [-10,1]
# C_range_y = [-12, 12]
# C_range_z = [-0.5, 8] # NOTE: Sensor could be below ship with negative range and capture CAD gaps. Manually remove them if the selected C is within the ship.

def spherical_to_cartesian(r, theta, phi):
    """ Convert spherical (r, theta, phi) to Cartesian (x, y, z). """
    x = r * np.cos(phi) * np.cos(theta)
    y = r * np.cos(phi) * np.sin(theta)
    z = r * np.sin(phi)
    return np.array([x, y, z])

def create_C_F():
    """
    Generate points C and F using spherical coordinates and store them in corresponding arrays.
    Store their connecting vectors CF.
    """
    C = []
    F = []
    
    for _ in range(num_samples):
        # Generate random polar coordinates for C (mu=0, sigma=1)
        r_c = np.random.uniform(r_C[0], r_C[1])
        theta_c = np.random.uniform(theta_B_C[0], theta_B_C[1])
        phi_c = np.random.uniform(phi_B_C[0], phi_B_C[1])
        C.append(spherical_to_cartesian(r_c, theta_c, phi_c))

        # Generate random polar coordinates for F (mu=0, sigma=1)
        r_f = np.random.uniform(r_F[0], r_F[1])
        theta_f = np.random.uniform(theta_B_F[0], theta_B_F[1])
        phi_f = np.random.uniform(phi_B_F[0], phi_B_F[1])
        F.append(spherical_to_cartesian(r_f, theta_f, phi_f))

    C = np.array(C)
    F = np.array(F)
    CF = F - C  # Vector from C to F

    return C, F, CF
    

def set_model_state():
    C, F, CF = create_C_F()

    print(f"Generated {len(C)} pairs of C and F points.")
    print("First C point:", C[0])
    print("First F point:", F[0])
    print("First CF vector:", CF[0])

    # Plot the points C blue and F red using pyvista
    for i in range(num_samples):
        

    # np.save("C_points.npy", C)
    # np.save("F_points.npy", F)
    # np.save("CF_vectors.npy", CF)

if __name__ == "__main__":
    try:
        set_model_state()
    except rospy.ROSInterruptException:
        pass