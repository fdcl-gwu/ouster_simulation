import numpy as np
import os
from scipy.spatial.transform import Rotation
from scipy.linalg import expm


np.random.seed(41)

num_samples_C = 1500
num_samples_F = 750

# Constants
theta_B_C = [np.pi/4, 7*np.pi/4,]
phi_B_C = [0, np.pi/3.5]
r_C = [0,12]

theta_B_F = [-np.pi/4, np.pi/4,]
phi_B_F = [-np.pi/8, np.pi/3]
r_F = [4,8]

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
    
def hat_map(v):
    """Compute the hat map (skew-symmetric matrix) of a 3D vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def to_quaternion(R):
    """Convert a rotation matrix to a quaternion."""
    q = np.empty(4)
    q[0] = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
    q[1] = (R[2,1] - R[1,2]) / (4*q[0])
    q[2] = (R[0,2] - R[2,0]) / (4*q[0])
    q[3] = (R[1,0] - R[0,1]) / (4*q[0])
    return q

def to_rotation_matrix(quaternion):
    rotation = Rotation.from_quat(quaternion)
    return rotation.as_matrix()

def get_R(C,F,CF):
    R_prime = np.eye(3)
    r1_prime = np.zeros((3,1))
    r2_prime = np.zeros((3,1))
    r3_prime = np.zeros((3,1))
    e_3 = np.array([0,0,1]) #only used to get r1_prime given LiDAR coordinate system
    psi = [-np.pi/6, np.pi/6]
    R_for_C = [] # for each C, contains the corresponding R

    # First use CF to compute R'. Then use R' to compute R.
    for i in range(len(CF)):
        r1_prime = CF[i]/np.linalg.norm(CF[i])
        r2_prime = np.cross(e_3, r1_prime)/np.linalg.norm(np.cross(e_3, r1_prime))
        r3_prime = np.cross(r1_prime, r2_prime)
        R_prime = np.column_stack((r1_prime, r2_prime, r3_prime))

        # Now compute R
        # psi = np.random.uniform(psi[0],psi[1]) # range of psi to be selected randomly
        psi = 0
        r1_prime_hat = hat_map(r1_prime) #TODO: not using e3_hat?
        # exp_matrix = expm(psi*r1_prime_hat)
        exp_matrix = np.eye(3) + np.sin(psi) * r1_prime_hat + (1 - np.cos(psi)) * np.dot(r1_prime_hat, r1_prime_hat) #explcitly use Rodrigues' formula
        R = np.dot(R_prime, exp_matrix)

        # append R to a list
        R_for_C.append(R)

    return R_for_C
    # TODO: plot the matrix in the plotter

def plot_points(C, F, CF, R_list):
    import pyvista as pv

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

    # Draw a coordiante axes for this R at location C in the plotter
    R = R_list[11]  # Use the first rotation matrix
    origin = C[11]  # Assume the first C point as the origin for the axes

    # Define axes scaled and transformed by R
    axes_length = 1.0
    # Add the axes to the plot
    plotter.add_arrows(origin, R[:, 0] * axes_length, color='red')    # X-axis
    plotter.add_arrows(origin, R[:, 1] * axes_length, color='green')  # Y-axis
    plotter.add_arrows(origin, R[:, 2] * axes_length, color='blue')   # Z-axis

    # Set labels and background
    plotter.set_background('white')
    plotter.add_axes(labels_off=False)
    plotter.show_grid()

    # Show the plot
    plotter.show(title='3D Visualization of PLY Point Cloud')


def generate_scatter_data():
    # Create a publisher to the /gazebo/set_model_state topic, sending over computed poses
    C, F = create_C_F()
    CF = generate_CF_vectors(C,F)

    print(f"Generated {len(C)} pairs of C and F points.")
    
    R_list = get_R(C,F,CF)
    return C, F, CF, R_list

# 29.01.2025: Requires using base conda env on Jetson.
if __name__ == "__main__":
    C, F, CF, R_list = generate_scatter_data()
    plot_points(C, F, CF, R_list)
