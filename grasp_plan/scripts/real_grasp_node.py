#!/usr/bin/env python3
#encoding: utf-8
import python3_in_ros

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from grasp_plan.msg import Grasp
from grasp_planner import GrapsPanler
from tf_mapper import TFMapper_EyeInHand
from camera import CameraBase
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path
from grasp_plan.msg import Force
from grasp_plan.msg import Plot
# from example_move_it_trajectories   import  ExampleMoveItTrajectories

plan_flag = False
finish_plan_flag = False 
# camera_intrinsic_path = Path.cwd().joinpath("grasp_plan/scripts/cameras/camera_intrinsic.npy")
# camera_extrinsic_path = Path.cwd().joinpath("grasp_plan/scripts/cameras/camera_extrinsic_new.npy")
camera_intrinsic_path = Path.cwd().joinpath("grasp_plan/scripts/cameras/real_camera_intrinsic.npy")
camera_extrinsic_path = Path.cwd().joinpath("grasp_plan/scripts/cameras/real_camera_extrinsic.npy")
camera_intrinsic = np.load(camera_intrinsic_path)
camera_extrinsic = np.load(camera_extrinsic_path)
counter = 0

def callback(data):   
    global publisher,grasp_planner,counter,plan_flag,listener,grasp_msg,plot_publisher,plot_msg

    ##########subcribe_stage###########

    if data.collision :
        plan_flag = True#
    else :
        plan_flag = False
        return

    ##########grasp_plan_stage###########   
    if plan_flag :  
        plan_flag = False
        try:
            data = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=0.5)
            rospy.loginfo("camera get")
        except:
            rospy.loginfo("!!!camera fault!!")
            return
        bridge = CvBridge()
        raw_depth = bridge.imgmsg_to_cv2(data)
        # cv2.imshow("frame" , raw_depth)
        # cv2.waitKey(3)
        raw_rgb = None
        raw_depth = raw_depth/1000      
        grasp_planer = GrapsPanler(raw_rgb, raw_depth)

        # if grasp_planer.is_ready():
        rospy.loginfo('\033[31;41m 规划预备!!!!!! \033[0m')
        grasp_describer =grasp_planer.get_grasp()
        rospy.loginfo('guihua完成 \033[0m')
        
        if grasp_describer is None:
            rospy.loginfo('\033[31;41m 规划失败!!!!!! \033[0m')
            
        else :
            print(grasp_describer.center, grasp_describer.angle,grasp_describer.depth)
            p0,p1,_,_=grasp_describer.endpoints()
            p0 = list(p0)
            p1 = list(p1)
            plot_msg.p0 = p0
            plot_msg.p1 = p1
            plot_publisher.publish(plot_msg)

            grasp_depth = grasp_planer.calculate_grasp_depth()
            grasp_describer.depth  = grasp_depth
            
            camera = CameraBase(camera_intrinsic,[300,300])
            tf_mapper = TFMapper_EyeInHand(camera,camera_extrinsic)
            try:
                (trans,rot) = listener.lookupTransform('base_link', 'end_effector_link', rospy.Time(0))
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            print("trans,rot",trans,rot)
            rotation_matrix = R.from_quat(rot).as_matrix()
            print("rotation_matrix",rotation_matrix)
            tcp2base = np.eye(4)
            tcp2base[:3,:3] = rotation_matrix
            tcp2base[:3,3] = trans
            print("tcp2base",tcp2base)
            g_base, width = tf_mapper.grasp2d_to_matrix(grasp_describer,tcp2base)
            print("g_world, width",g_base, width)
            # plot_grasp(img=raw_depth, g=grasp_describer)
            # a = np.array([[0,0,1],[1,0,0],[0,1,0]])
            quat = R.from_matrix(g_base[:3,:3]).as_quat()#g_base[:3,:3]
            RPY = R.from_matrix(g_base[:3,:3]).as_euler('xyz')
            tran = g_base[:3,3]
            finish_plan_flag = True


        #########publish_stage###########        
            excute_flag =input("plan ready,excute now (y/n): ")
            if excute_flag=="y":
                grasp_msg.position_x = tran[0]
                grasp_msg.position_y = tran[1]
                grasp_msg.position_z = tran[2]
                grasp_msg.orientation_x = quat[0]
                grasp_msg.orientation_y = quat[1]
                grasp_msg.orientation_z = quat[2]
                grasp_msg.orientation_w = quat[3]
                grasp_msg.angle_x = RPY[0]/3.1415926*180
                grasp_msg.angle_y = RPY[1]/3.1415926*180
                grasp_msg.angle_z = RPY[2]/3.1415926*180
                grasp_msg.translation_speed=0.08#0.8 # 米
                grasp_msg.orientation_speed=10#15 # 度
                publisher.publish(grasp_msg)
                print("publish execution")
            else:
                print("give up the execution")
            
            # success = moveiter.is_init_success
            # init_pose = moveiter.get_cartesian_pose()
            # if success:
            #     rospy.loginfo("Reaching Cartesian Pose...")
            #     target_pose = moveiter.get_cartesian_pose()

            #     success &= moveiter.reach_cartesian_pose(pose=target_pose, tolerance=0.01, constraints=None)
            #     print ("是否执行成功",success)
            # else:
            #     print ("moveiter 初始化失败")
            
            # if moveiter.is_gripper_present and success:
            #     rospy.loginfo("Opening the gripper...")
            #     success &= moveiter.reach_gripper_position(0)
            #     print (success)
            
            # if success:
            #     success &= moveiter.reach_cartesian_pose(pose=init_pose, tolerance=0.01, constraints=None)
            #     print ("是否返回初始状态成功",success)

    else:
        rospy.loginfo("no plan")
        return



    
    

 
def displayWebcam():
 
    # make a video_object and init the video object
    global publisher,grasp_planner,listener,grasp_msg,plot_publisher,plot_msg
    
    # ROS节点初始化
    rospy.init_node('camera_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    # rospy.Subscriber("/d435/depth/image_raw", Image, callback)
    rospy.Subscriber("/my_gen3/temp", Force, callback)
    publisher = rospy.Publisher('/grasp/plan', Grasp, queue_size=1)
    plot_publisher = rospy.Publisher('/grasp/plot', Plot, queue_size=1)
    listener = tf.TransformListener()
    # moveiter = ExampleMoveItTrajectories()
    grasp_msg = Grasp()
    plot_msg = Plot()
    rospy.spin()


def plot_grasp(img ,g, offset=[0, 0]):
        """ 使用plt在图像上展示一个夹爪 """
        def plot_2p(p0, p1, mode='r', width=None):
            p0 -= offset
            p1 -= offset
            x = [p0[0], p1[0]]
            y = [p0[1], p1[1]]
            plt.plot(x, y, mode, linewidth=width)

        def plot_center(center, axis, length, mode='r', width=2):
            axis = axis / np.linalg.norm(axis)
            p0 = center - axis * length / 2
            p1 = center + axis * length / 2
            plot_2p(p0, p1, mode, width)

        plt.clf()
        plt.axis('off')
        plt.imshow(img)
        p0, p1, _, _ = g.endpoints
        axis = [g.axis[1], -g.axis[0]]
        plot_2p(p0, p1, 'r--', width=5)
        plot_center(p0, axis, g.width_px/5, width=8)
        plot_center(p1, axis, g.width_px/5, width=8)
        plt.plot(*(g.center - offset), 'bo')
        plt.show()






 
if __name__ == '__main__':
    displayWebcam()
