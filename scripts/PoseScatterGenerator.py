import numpy as np
import os
from scipy.spatial.transform import Rotation
from scipy.linalg import expm


np.random.seed(41)

num_samples_C = 20000
num_samples_F =20000

# Constants
theta_B_C = [np.pi/3.5, 6*np.pi/3.5]
phi_B_C = [0, np.pi/2.5]
r_C = [0,20]
r_c_mean = 6
r_c_std = 5

theta_B_F = [-np.pi/5, np.pi/5,]
phi_B_F = [-np.pi/8, np.pi/3]
r_F = [0,17]

def spherical_to_cartesian(r, theta, phi):
    """ Convert spherical (r, theta, phi) to Cartesian (x, y, z). """
    x = r * np.cos(phi) * np.cos(theta)
    y = r * np.cos(phi) * np.sin(theta)
    z = r * np.sin(phi)
    return np.array([x, y, z])

# Generate vectors CF connecting C and F points. For each F point, there are two C points.
# NOTE: Assumes there are twice as many C points as F points
def generate_CF_vectors(C, F, num_orientations=1):
    CF = []
    F_updated = []
    C_repeated = []

    for i in range(len(C)):
        c_point = C[i]
        z_min = c_point[2] - 1.2
        z_max = c_point[2] + 1.2

        # Check if special rule applies (based on C's x and y)
        if (abs(c_point[1]) > 3) and (-3 < c_point[0] < 2):
            # F near the origin in x, y, but within z bounds of current C
            F_filtered = F[
                (np.abs(F[:, 0] - 2.0) < 1.0) &  # x near 2
                (np.abs(F[:, 1]) < 1.0) &        # y near 0
                (F[:, 2] >= z_min) & (F[:, 2] <= z_max)
            ]
        else:
            # Default filtering based on similar z
            F_filtered = F[
                (F[:, 2] >= z_min) & (F[:, 2] <= z_max)
            ]

        # Fallback if none found
        if len(F_filtered) == 0:
            F_selected = F[np.argmin(np.abs(F[:, 2] - c_point[2]))]
            for _ in range(num_orientations):
                CF.append(F_selected - c_point)
                F_updated.append(F_selected)
                C_repeated.append(c_point)
            continue

        # Sample orientations
        for _ in range(num_orientations):
            F_selected = F_filtered[np.random.randint(0, len(F_filtered))]
            CF.append(F_selected - c_point)
            F_updated.append(F_selected)
            C_repeated.append(c_point)

    return np.array(CF), np.array(F_updated), np.array(C_repeated)


def create_C_F():
    """
    Generate points C and F using spherical coordinates and store them in corresponding arrays.
    Store their connecting vectors CF.
    """

    C = []
    F = []

    # Sample first half of C in Cartesian box
    while len(C) < num_samples_C/3:
        x = np.random.uniform(-8, 2.3)
        y = np.random.uniform(-10, 10)
        z = np.random.uniform(0.26, 5)
        point = np.array([x, y, z])

        if point[2] >= 0.26:  # Keep this condition if needed
            C.append(point)

    # Sample other half of C in spherical coordinates
    while len(C) < num_samples_C:
        r_c = np.random.normal(r_c_mean, r_c_std)
        if not r_C[0] <= r_c <= r_C[1]:
            continue
        theta_c = np.random.uniform(theta_B_C[0], theta_B_C[1])
        phi_c = np.random.uniform(phi_B_C[0], phi_B_C[1])
        point = spherical_to_cartesian(r_c, theta_c, phi_c)
        if point[2] >= 0.26:
            C.append(point)

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

def get_R(CF):
    R_prime = np.eye(3)
    r1_prime = np.zeros((3,1))
    r2_prime = np.zeros((3,1))
    r3_prime = np.zeros((3,1))
    e_1 = np.array([1,0,0])
    e_3 = np.array([0,0,1]) #only used to get r1_prime given LiDAR coordinate system
    psi_range = [-np.pi/18, np.pi/18]
    R_for_C = [] # for each C, contains the corresponding R

    # First use CF to compute R'. Then use R' to compute R.
    for i in range(len(CF)):
        r1_prime = CF[i]/np.linalg.norm(CF[i])
        r2_prime = np.cross(e_3, r1_prime)/np.linalg.norm(np.cross(e_3, r1_prime))
        r3_prime = np.cross(r1_prime, r2_prime)
        R_prime = np.column_stack((r1_prime, r2_prime, r3_prime))

        # Now compute R
        psi = np.random.uniform(psi_range[0],psi_range[1]) # range of psi to be selected randomly
        # psi = 0
        e1_prime_hat = hat_map(e_1)
        exp_matrix = expm(psi*e1_prime_hat)
        # exp_matrix = np.eye(3) + np.sin(psi) * r1_prime_hat + (1 - np.cos(psi)) * np.dot(r1_prime_hat, r1_prime_hat) #explcitly use Rodrigues' formula
        R = np.dot(R_prime, exp_matrix)

        # append R to a list
        R_for_C.append(R)

    return R_for_C
    # TODO: plot the matrix in the plotter
def plot_points(C, F, CF, R_list):
    import pyvista as pv

    stl_path = "/home/fdcl/Ouster/gazebo_ws_fdcl/src/ouster_simulation/ouster_description/meshes/rotated_ship.stl"
    mesh = pv.read(stl_path)
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color='lightgrey', opacity=0.5)

    # Add all C and F points once as point clouds
    plotter.add_points(C, color='blue', point_size=3, render_points_as_spheres=True)
    plotter.add_points(F, color='red', point_size=3, render_points_as_spheres=True)

    # Plot only a subset to avoid cluttering the view
    num_lines = min(100, len(C))
    axes_length = 1.0

    for i in range(num_lines):
        plotter.add_lines(np.array([C[i], F[i]]), color='purple', width=2)

        R = R_list[i]
        origin = C[i]
        plotter.add_arrows(origin, R[:, 0] * axes_length, color='red')    # X
        plotter.add_arrows(origin, R[:, 1] * axes_length, color='green')  # Y
        plotter.add_arrows(origin, R[:, 2] * axes_length, color='blue')   # Z

    plotter.set_background('white')
    plotter.add_axes(labels_off=False)
    plotter.show_grid()
    plotter.show(title='3D Visualization of 1 Orientations per C Point')



def generate_scatter_data(num_orientations=1):
    C, F = create_C_F()
    CF, F_updated, C_repeated = generate_CF_vectors(C, F, num_orientations=num_orientations)

    print(f"Generated {len(C_repeated)} pairs of C and F points.")

    R_list = get_R(CF)
    return C_repeated, F_updated, CF, R_list


# 29.01.2025: Requires using base conda env on Jetson.
if __name__ == "__main__":
    C, F, CF, R_list = generate_scatter_data(num_orientations=1)
    plot_points(C, F, CF, R_list)

