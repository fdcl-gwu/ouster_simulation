
import numpy as np

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

# Example usage
matrix1 = np.array([[-5.02464388e-02,  9.98735507e-01,  1.63768012e-03],
                    [9.98195271e-01,  5.02731177e-02, -3.28453185e-02],
                    [-3.28861171e-02, -1.56357310e-05, -9.99459105e-01]])

matrix2 = np.array([[1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1]])

result = multiply_so3_matrices(matrix1, matrix2)
print(result)