'''
@Description: In User Settings Edit
@Author: Lai
@Date: 2019-11-04 13:14:06
@LastEditTime : 2020-01-04 21:05:41
@LastEditors  : Lai
'''
import os
import cv2
import trimesh
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation
from glob import glob
import numpy as np
file_path = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.abspath(os.path.join(file_path, '..'))


x = Rotation.from_euler('x', 45, degrees=True).as_matrix()
z = Rotation.from_euler('z', 45, degrees=True).as_matrix()
BASE2WORLD = np.eye(4)
BASE2WORLD[:3, :3] = x.dot(z)
BASE2WORLD[:3, 3] = [0, -0.225, 0.4]
WORLD2BASE = np.linalg.inv(BASE2WORLD)
# print(BASE2WORLD)


def display(camera_m, eye_in_hand=False):
    base2world = BASE2WORLD if not eye_in_hand else np.array([[0, 1, 0, 0],
                                                              [1, 0, 0, 0],
                                                              [0, 0, -1, 1],
                                                              [0, 0, 0, 1]])

    source_axis = trimesh.creation.axis(origin_color=(255, 0, 0), transform=np.eye(4))
    base_axis = trimesh.creation.axis(origin_color=(0, 255, 0), transform=base2world)
    camera_axis = trimesh.creation.axis(origin_color=(
        0, 0, 255), transform=base2world.dot(camera_m))
    scene = trimesh.scene.scene.Scene()
    scene.add_geometry(base_axis)
    scene.add_geometry(source_axis)
    scene.add_geometry(camera_axis)
    scene.show()


def display1(camera_m, eye_in_hand=False):
    base2world = BASE2WORLD
    bb = BASE2WORLD.copy()
    bb[:3, :2] = -bb[:3, :2]
    source_axis = trimesh.creation.axis(origin_color=(255, 0, 0), transform=np.eye(4))
    base_axis = trimesh.creation.axis(origin_color=(0, 255, 0), transform=base2world)
    camera_axis = trimesh.creation.axis(origin_color=(
        0, 0, 255), transform=bb)
    scene = trimesh.scene.scene.Scene()
    scene.add_geometry(base_axis)
    scene.add_geometry(source_axis)
    scene.add_geometry(camera_axis)
    scene.show()


def to_matrix(pos):
    print("ok",pos)
    r = Rotation.from_rotvec(pos[-3:]).as_matrix()
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = pos[:3]
    return m


def to_vector(m):
    pos = np.zeros((6,))
    pos[-3:] = Rotation.from_matrix(m[:3, :3]).as_rotvec()
    pos[:3] = m[:3, 3]
    return pos


def eye_arm_calibrate(images, urs,  mtx, dist, eye_in_hand=False, show=True):
    # 创建标定板模型
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
    parameters = aruco.DetectorParameters_create()
    board = aruco.GridBoard_create(2, 2, 0.08, 0.01, aruco_dict)
    # 从图像中得到标定板坐标系到相机坐标系的变换
    cam = []
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            gray, aruco_dict,  parameters=parameters)
        retval, rvec, tvec = aruco.estimatePoseBoard(
            corners, ids, board, mtx, dist, None, None)
        cam.append(to_matrix(np.squeeze(np.r_[tvec, rvec])))
    # urs是工具坐标系到基座坐标系的变换,在eye to hand的时候用的是基座坐标到手坐标的变换,要求逆
    # urs = [to_matrix(s) for s in urs]##在外面收集的时候就已经是按旋转矩阵格式来收集，所以此处不需要再变换
    if not eye_in_hand:
        urs = [np.linalg.inv(s) for s in urs]
    # calibrateHandEye函数的输入要分开旋转矩阵和平移向量
    R_gripper2base = np.array([pos[:3, :3] for pos in urs])
    t_gripper2base = np.array([pos[:3, 3] for pos in urs])
    R_target2cam = np.array([pos[:3, :3] for pos in cam])
    t_target2cam = np.array([pos[:3, 3] for pos in cam])

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base,
        R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_PARK)
    # 把输出还原为4x4变换矩阵
    cam2base = np.eye(4)
    cam2base[:3, :3] = np.squeeze(R_cam2gripper)
    cam2base[:3, 3] = np.squeeze(t_cam2gripper)
    print(cam2base)
    if show:
        display(cam2base, eye_in_hand)
    return cam2base


def calibration_in_file(file_path, mtx_path, eye_in_hand=False):
    files = glob(os.path.join(file_path, '*.jpg'))
    mtx = np.load(os.path.join(mtx_path, 'mtx.npy'))
    dist = np.load(os.path.join(mtx_path, 'dist.npy'))
    images = []
    urs = []
    for f in files:
        npy_f = os.path.splitext(f)[0]+'.npy'
        img = cv2.imread(f)
        ur = np.load(npy_f)
        urs.append(ur)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        images.append(img)
    eye_arm_calibrate(images, urs, mtx, dist, eye_in_hand)


if __name__ == "__main__":
    # mtx_path = os.path.join(root_path, 'images/realsense')
    # out_path = os.path.join(root_path, 'arm_images/realsense')
    mtx_path = os.path.join(root_path, 'cameras/images/primesense')
    out_path = os.path.join(root_path, 'cameras/arm_images/primesense')
    calibration_in_file(out_path, mtx_path, False)
    # display1(None)
