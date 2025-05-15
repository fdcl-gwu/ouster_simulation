import numpy as np
import pyvista as pv

# Load the scaled STL mesh
mesh = pv.read('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/rotated_scaled_ShipBlenderCycles_final.stl')

# Load the aligned YP point cloud
yp_cloud = np.loadtxt('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/YP_cloud.txt')
pcd_yp = pv.PolyData(yp_cloud)

# Load the STL-derived point cloud
stl_cloud = np.loadtxt('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/STL_cloud.txt')
pcd_stl = pv.PolyData(stl_cloud)

# Set up plotter
plotter = pv.Plotter()
plotter.show_grid()

# Add mesh
plotter.add_mesh(mesh, color='lightgrey', show_edges=False, opacity=0.8)

# Add aligned YP cloud
plotter.add_points(pcd_yp, color='red', point_size=5, render_points_as_spheres=True)

# Add STL-derived point cloud
plotter.add_points(pcd_stl, color='blue', point_size=5, render_points_as_spheres=True)

# Add axes and show
plotter.add_axes_at_origin()
plotter.show()
