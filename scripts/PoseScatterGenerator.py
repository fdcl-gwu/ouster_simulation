import numpy as np
import os
from scipy.spatial.transform import Rotation
from scipy.linalg import expm


np.random.seed(42)

num_samples_C = 4000
num_samples_F = 4000

# Constants
theta_B_C = [np.pi/3.5, 6*np.pi/3.5]
phi_B_C = [0, np.pi/3]
r_C = [0,20]
r_c_mean = 6
r_c_std = 5

theta_B_F = [-np.pi/5, np.pi/5,]
phi_B_F = [-np.pi/8, np.pi/3]
r_F = [2,17]

def spherical_to_cartesian(r, theta, phi):
    """ Convert spherical (r, theta, phi) to Cartesian (x, y, z). """
    x = r * np.cos(phi) * np.cos(theta)
    y = r * np.cos(phi) * np.sin(theta)
    z = r * np.sin(phi)
    return np.array([x, y, z])

# Generate vectors CF connecting C and F points. For each F point, there are two C points.
# NOTE: Assumes there are twice as many C points as F points
def generate_CF_vectors(C, F, num_orientations=1):
    def elevation_angle(v):
        return np.arcsin(v[2] / np.linalg.norm(v))

    def filter_F_by_z(F_pool, z_center, z_margin=1.2):
        return F_pool[(F_pool[:, 2] >= z_center - z_margin) & (F_pool[:, 2] <= z_center + z_margin)]

    def sample_valid_direction(c_point, F_candidates, angle_threshold_rad):
        for _ in range(100):
            F_sample = F_candidates[np.random.randint(len(F_candidates))]
            direction = F_sample - c_point
            if abs(elevation_angle(direction)) <= angle_threshold_rad:
                return direction, F_sample
        return None, None

    CF = []
    F_updated = []
    C_repeated = []

    angle_threshold_rad = np.deg2rad(15)

    for c_point in C:
        use_origin_sampling = False

        # Midship logic only if y is not near center
        if -1 < c_point[0] < 10 and abs(c_point[1]) > 2:
            use_origin_sampling = True

        if use_origin_sampling:
            # Use origin-near F
            F_near_origin = F[
                (F[:, 0] >= 2.0) & (F[:, 0] <= 3.0) &
                (np.abs(F[:, 1]) < 1.0)
            ]
            F_filtered = filter_F_by_z(F_near_origin, c_point[2])

            if len(F_filtered) > 0:
                test_dir = F_filtered[np.random.randint(len(F_filtered))] - c_point
                if abs(elevation_angle(test_dir)) > angle_threshold_rad:
                    F_filtered = filter_F_by_z(F, c_point[2])
        else:
            F_filtered = filter_F_by_z(F, c_point[2])

        if len(F_filtered) == 0:
            F_selected = F[np.argmin(np.abs(F[:, 2] - c_point[2]))]
            for _ in range(num_orientations):
                CF.append(F_selected - c_point)
                F_updated.append(F_selected)
                C_repeated.append(c_point)
            continue

        for _ in range(num_orientations):
            direction, F_selected = sample_valid_direction(c_point, F_filtered, angle_threshold_rad)
            if direction is not None:
                CF.append(direction)
                F_updated.append(F_selected)
                C_repeated.append(c_point)
            else:
                # fallback even if steep
                F_selected = F_filtered[0]
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
    from scipy.stats import truncnorm

    def sample_truncated_normal(mean, std, lower, upper, size=1):
        a, b = (lower - mean) / std, (upper - mean) / std
        return truncnorm.rvs(a, b, loc=mean, scale=std, size=size)

    while len(C) < num_samples_C / 4:
        x = sample_truncated_normal(mean=0, std=3, lower=-7, upper=2.3)[0]
        y = sample_truncated_normal(mean=0, std=3, lower=-7, upper=7)[0]
        z = np.random.uniform(0.26, 6)
        point = np.array([x, y, z])
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
    psi_range = [-np.pi/24, np.pi/24]
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
    num_lines = min(200, len(C))
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
    def compute_elevation_angle(v):
        """Returns elevation angle in degrees between vector and xy-plane."""
        return np.degrees(np.arcsin(v[2] / np.linalg.norm(v)))

    # Debugging: Count vectors with elevation > 15°
    high_elevation_count = 0
    for i in range(len(CF)):
        elev = compute_elevation_angle(CF[i])
        if abs(elev) > 15:
            high_elevation_count += 1
            print(f"CF[{i}] elevation = {elev:.2f} degrees (C: {C_repeated[i]}, F: {F_updated[i]})")

    print(f"Total CF vectors with elevation > 15°: {high_elevation_count}/{len(CF)}")

    print(f"Generated {len(C_repeated)} pairs of C and F points.")

    R_list = get_R(CF)
    return C_repeated, F_updated, CF, R_list


# 29.01.2025: Requires using base conda env on Jetson.
if __name__ == "__main__":
    C, F, CF, R_list = generate_scatter_data(num_orientations=1)
    plot_points(C, F, CF, R_list)

