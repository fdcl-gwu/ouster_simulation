import numpy as np
from scipy.spatial.transform import Rotation

def quaternion_to_rotation_matrix(q):
    # Convert quaternion (x, y, z, w) to (w, x, y, z) format
    q_wxyz = [q[3], q[0], q[1], q[2]]

    # Create rotation object from quaternion
    rotation = Rotation.from_quat(q_wxyz)

    # Get the rotation matrix
    rotation_matrix = rotation.as_matrix()

    return rotation_matrix

def create_transformation_matrix(position, orientation):
    # Create rotation object from quaternion (x, y, z, w)
    rotation = Rotation.from_quat(orientation)

    # Get the rotation matrix
    rotation_matrix = rotation.as_matrix()

    # Construct the transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = position

    return transformation_matrix

def multiply_so3_matrices(matrix1, matrix2):
    """
    Multiplies two SO(3) rotation matrices together.

    Parameters:
    matrix1 (numpy.ndarray): The first 3x3 rotation matrix.
    matrix2 (numpy.ndarray): The second 3x3 rotation matrix.

    Returns:
    numpy.ndarray: The resulting 3x3 rotation matrix.
    """
    return np.dot(matrix1, matrix2)


if __name__ == "__main__":
    
    
    # ############# MATMULT #############
    # matrix1 = np.array([[-5.02464388e-02,  9.98735507e-01,  1.63768012e-03],
    #                     [9.98195271e-01,  5.02731177e-02, -3.28453185e-02],
    #                     [-3.28861171e-02, -1.56357310e-05, -9.99459105e-01]])

    # matrix2 = np.array([[1, 0, 0],
    #                     [0, -1, 0],
    #                     [0, 0, -1]])

    # result = multiply_so3_matrices(matrix1, matrix2)
    # print(result)

    # ############# TOPIC 2 ROT #############
    # position = [1.86964867983716, -3.028062835607197, 2.7451070230083827]
    # orientation = [-0.7227815985679626, 0.0, 0.0, 0.6910766363143921]

    # # Call the function to create the transformation matrix
    # transformation_matrix = create_transformation_matrix(position, orientation)

    # # Print the transformation matrix
    # print("Transformation Matrix:")
    # print(transformation_matrix)

    # ############# QUAT 2 ROT #############
    # q = [0, 0, 1, 0]

    # # Convert quaternion to rotation matrix
    # rotation_matrix = quaternion_to_rotation_matrix(q)

    # # Print the rotation matrix
    # print("Rotation Matrix:")
    # print(rotation_matrix)