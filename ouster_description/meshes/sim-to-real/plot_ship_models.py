import numpy as np
import pyvista as pv

# Load STL and YP point clouds
stl_cloud = np.loadtxt('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/STL_cloud.txt')  # blue, reference
yp_cloud = np.loadtxt('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/YP_cloud.txt')    # red, aligned

# Create PyVista point cloud objects
pcd_stl = pv.PolyData(stl_cloud)
pcd_yp = pv.PolyData(yp_cloud)

# Set up plotter
plotter = pv.Plotter()
plotter.show_grid()
plotter.add_points(pcd_stl, color='blue', point_size=5, render_points_as_spheres=True)
plotter.add_points(pcd_yp, color='red', point_size=5, render_points_as_spheres=True)
plotter.add_axes_at_origin()
plotter.show()
