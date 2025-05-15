import numpy as np
import pyvista as pv
import small_gicp  # adjust this import if needed
import open3d as o3d

# Load the STL file as both Open3D and PyVista objects
stl_mesh_o3d = o3d.io.read_triangle_mesh("/Users/karlsimon/Downloads/rotated_ShipBlenderCycles_final.stl")
stl_mesh_pv = pv.read("/Users/karlsimon/Downloads/rotated_ShipBlenderCycles_final.stl")

# Compute centroid ONCE from PyVista mesh
centroid = stl_mesh_pv.points.mean(axis=0)

# Scale the mesh (same scale factor as used for the point cloud)
scale_factor = 1.015
stl_mesh_pv.points = (stl_mesh_pv.points - centroid) * scale_factor + centroid

# Save the scaled mesh to a new STL file
scaled_stl_path = "/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/rotated_scaled_ShipBlenderCycles_final.stl"
stl_mesh_pv.save(scaled_stl_path)

# Sample the (unscaled) Open3D mesh to a point cloud
cloud1 = stl_mesh_o3d.sample_points_uniformly(number_of_points=50000)

# Apply same scaling to the sampled point cloud using the same centroid
points = np.asarray(cloud1.points)
points = (points - centroid) * scale_factor + centroid
cloud1 = points  # now a NumPy array

# Save as STL_cloud.txt
np.savetxt('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/STL_cloud.txt', cloud1, fmt='%.6f')

# Load YP ship point cloud
cloud2 = np.loadtxt('/Users/karlsimon/Downloads/12_jetson_files/dlio_deskewed/yp_complete_cloud_less_dense.txt')

# Define rotation matrix (for optional alignment)
theta = np.deg2rad(-1.5)
R_y = np.array([
    [np.cos(theta), 0, np.sin(theta), 0],
    [0, 1, 0, 0],
    [-np.sin(theta), 0, np.cos(theta), 0],
    [0, 0, 0, 1]
])

# Run small_gicp alignment
result = small_gicp.align(
    target_points=cloud1,
    source_points=cloud2,
    init_T_target_source=R_y,
    registration_type='GICP',
    downsampling_resolution=0.05,
    max_correspondence_distance=0.8,
    num_threads=4
)

# Apply transformation to cloud2
cloud2_aligned = (result.T_target_source[:3, :3] @ cloud2.T).T + result.T_target_source[:3, 3]

# Save as YP_cloud.txt
np.savetxt('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/YP_cloud.txt', cloud2_aligned, fmt='%.6f')

# Plot both point clouds
plotter = pv.Plotter()
plotter.show_grid()
plotter.add_points(pv.PolyData(cloud1), color='blue', point_size=5, render_points_as_spheres=True)
plotter.add_points(pv.PolyData(cloud2_aligned), color='red', point_size=5, render_points_as_spheres=True)
plotter.add_axes_at_origin()
plotter.show()
