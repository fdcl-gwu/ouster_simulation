import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import numpy as np
import os
import pyvista as pv

np.random.seed(42)

num_samples_C = 1500
num_samples_F = 750

# Constants
theta_B_C = [np.pi/4, 7*np.pi/4,]
phi_B_C = [0, np.pi/3.5]
r_C = [0,12]

theta_B_F = [-np.pi/4, np.pi/4,]
phi_B_F = [-np.pi/8, np.pi/3]
r_F = [2.5,6]

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

# Generate vectors CF connecting C and F points. For each F point, there are two C points.
# NOTE: Assumes there are twice as many C points as F points
def generate_CF_vectors(C, F):
    CF = []
    for f in range(len(F)):
        c1 = C[2*f]
        c2 = C[2*f+1]
        CF.append(F[f]-c1)
        CF.append(F[f]-c2)
    CF = np.array(CF)
    return CF

def create_C_F():
    """
    Generate points C and F using spherical coordinates and store them in corresponding arrays.
    Store their connecting vectors CF.
    """
    C = []
    F = []
    
    for _ in range(num_samples_C):
        # Generate random polar coordinates for C (mu=0, sigma=1)
        r_c = np.random.uniform(r_C[0], r_C[1])
        theta_c = np.random.uniform(theta_B_C[0], theta_B_C[1])
        phi_c = np.random.uniform(phi_B_C[0], phi_B_C[1])
        C.append(spherical_to_cartesian(r_c, theta_c, phi_c))

    for _ in range(num_samples_F):
        # Generate random polar coordinates for F (mu=0, sigma=1)
        r_f = np.random.uniform(r_F[0], r_F[1])
        theta_f = np.random.uniform(theta_B_F[0], theta_B_F[1])
        phi_f = np.random.uniform(phi_B_F[0], phi_B_F[1])
        F.append(spherical_to_cartesian(r_f, theta_f, phi_f))

    C = np.array(C)
    F = np.array(F)    

    return C, F
    
def plot_points(C, F, CF):

    # Path to the STL file
    stl_path = "/home/fdcl/Ouster/gazebo_ws_fdcl/src/ouster_simulation/ouster_description/meshes/rotated_ship.stl"
    mesh = pv.read(stl_path)
    # Plot the points C blue and F red using pyvista
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color='lightblue', opacity=0.5)

    # add C, F points to the plot
    for i in range(num_samples_F):
        plotter.add_points(C[2*i], color='blue', point_size=3)
        plotter.add_points(C[2*i+1], color='blue', point_size=3)
        plotter.add_points(F[i], color='red', point_size=3)

    # select first num_lines pairs of points and draw lines between them
    num_lines = 10
    for i in range(num_lines):
        points = np.array([C[i], F[i//2]])
        plotter.add_lines(points, color='purple', width=2)

    # Set labels and background
    plotter.set_background('white')
    plotter.add_axes(labels_off=False)
    plotter.show_grid()

    # Show the plot
    plotter.show(title='3D Visualization of PLY Point Cloud')

def hat_map(v):
    """Compute the hat map (skew-symmetric matrix) of a 3D vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def get_R(C,F,CF):
    # First use CF to compute R'. Then use R' to compute R.
    R_prime = np.eye(3)
    r1_prime = np.zeros((3,1))
    r2_prime = np.zeros((3,1))
    r3_prime = np.zeros((3,1))
    e_3 = np.array([0,0,1]) #only used to get 

    for i in range(len(CF)):
        r1_prime = CF[i]/np.linalg.norm(CF[i])
        r2_prime = np.cross(e_3, r1_prime)/np.linalg.norm(np.cross(e_3, r1_prime))
        r3_prime = np.cross(r1_prime, r2_prime)
        R_prime = np.column_stack((r1_prime, r2_prime, r3_prime))

        # Now compute R
        psi = [-np.pi/6, np.pi/6] # range of psi to be selected randomly
        r1_prime_hat = hat_map(r1_prime) #TODO: not using e3_hat?
        
    # TODO: lot the matrix in the plotter


def set_model_state():
    # Create a publisher to the /gazebo/set_model_state topic, sending over computed poses
    C, F = create_C_F()
    CF = generate_CF_vectors(C,F)

    print(f"Generated {len(C)} pairs of C and F points.")
    
    plot_points(C, F, CF)
    # get_R(C,F,CF)

if __name__ == "__main__":
    try:
        set_model_state()
    except rospy.ROSInterruptException:
        pass