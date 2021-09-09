
import numpy as np
from numpy.core.defchararray import translate
import math



def eulerAngles2rotationMat(theta):
    """
    Calculates Rotation Matrix given euler angles.
    :param theta: 1-by-3 list [rx, ry, rz] angle in degree
    :return:
    RPY角，是ZYX欧拉角，依次 绕定轴XYZ转动[rx, ry, rz]
    """
    # if format is 'degree':
    #     theta = [i * math.pi / 180.0 for i in theta]
 
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
    
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
 
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    return np.dot(R_z, np.dot(R_y, R_x))


# xyz :0 0.05639 -0.00305
# rpy="3.14159265358979 3.14159265358979 0"

# camera matrix
# 360.01333 0.00000 243.87228
# 0.00000 360.013366699 137.9218444 
# 0.00000 0.00000 1.00000

rpy = np.array([3.14159265358979,3.14159265358979, 0])
translation = np.array([0, 0.05639, -0.00305])
camera_extrinsic = np.eye(4)
print(eulerAngles2rotationMat(rpy))
camera_extrinsic[:3,:3] = eulerAngles2rotationMat(rpy)
camera_extrinsic[:3,3] = translation
np.save("real_camera_extrinsic.npy",camera_extrinsic)
# camera_intrinsic = np.array([[695.9951171875, 0.0, 640.0],[0.0, 695.9951171875, 360.0],[ 0.0, 0.0, 1.0]])
camera_intrinsic = np.array([[360.01333 ,0.00000 ,243.87228],[0.00000 ,360.013366699 ,137.9218444],[ 0.0, 0.0, 1.0]])
np.save("real_camera_intrinsic.npy",camera_intrinsic)