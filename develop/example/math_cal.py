
import numpy as np
from scipy.spatial.transform import Rotation as R

# orient_list = [orient[3], orient[0], orient[1], orient[2]]
def M2O(matrix):
    shape = matrix.shape
    if shape == (4,4):
        matrix = np.array(matrix)
        orient = R.from_matrix(matrix[:-1][:, :-1]).as_quat()
        return orient
    elif shape == (3,3):
        matrix = np.array(matrix)
        orient = R.from_matrix(matrix).as_quat()
        return orient


def M2P(matrix):
    matrix = np.array(matrix)
    pos = matrix[0:3, 3].tolist()
    return pos