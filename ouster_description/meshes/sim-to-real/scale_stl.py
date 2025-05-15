import pyvista as pv
import numpy as np

# Load the STL mesh
mesh = pv.read('/Users/karlsimon/Documents/ppsurf_files/rotated_Ship.stl')

# Compute the centroid of the mesh
centroid = mesh.center

# Define scaling factor
scale_factor = 1.02

# Translate to origin, scale, then translate back
mesh.points = (mesh.points - centroid) * scale_factor + centroid

# (Optional) Save the scaled mesh to a new STL file
mesh.save('/Users/karlsimon/Downloads/stl_cloud_processing/sim-to-real/rotated_Ship_scaled.stl')

# Plot the scaled mesh
plotter = pv.Plotter()
plotter.show_grid()
plotter.add_mesh(mesh, color='blue', show_edges=False)
plotter.add_axes_at_origin()
plotter.show()
