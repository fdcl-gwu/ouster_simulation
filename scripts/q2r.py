
import numpy as np
from scipy.spatial.transform import Rotation

# Given quaternion values
q = [0, 0, 1, 0]

# Convert quaternion (x, y, z, w) to (w, x, y, z) format
q_wxyz = [q[3], q[0], q[1], q[2]]

# Create rotation object from quaternion
rotation = Rotation.from_quat(q_wxyz)

# Get the rotation matrix
rotation_matrix = rotation.as_matrix()

# Print the rotation matrix
print("Rotation Matrix:")
print(rotation_matrix)