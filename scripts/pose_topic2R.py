import numpy as np
from scipy.spatial.transform import Rotation

# Given position and orientation values
position = [1.86964867983716, -3.028062835607197, 2.7451070230083827]
orientation = [-0.7227815985679626, 0.0, 0.0, 0.6910766363143921]

# Create rotation object from quaternion (x, y, z, w)
rotation = Rotation.from_quat(orientation)

# Get the rotation matrix
rotation_matrix = rotation.as_matrix()

# Construct the transformation matrix
transformation_matrix = np.eye(4)
transformation_matrix[:3, :3] = rotation_matrix
transformation_matrix[:3, 3] = position

# Print the transformation matrix
print("Transformation Matrix:")
print(transformation_matrix)