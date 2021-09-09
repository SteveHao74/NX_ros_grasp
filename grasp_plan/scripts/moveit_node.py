#!/usr/bin/env python
#encoding: utf-8
import rospy
# from sensor_msgs.msg import Image
from grasp_plan.msg import Grasp
from example_move_it_trajectories   import  ExampleMoveItTrajectories


def callback(grasp_msg): 
    global init_pose
    success = moveiter.is_init_success
    # init_pose = moveiter.get_cartesian_pose()
    if moveiter.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= moveiter.reach_gripper_position(0)
        if success:    
            rospy.loginfo("开爪成功") 
        else :
            rospy.loginfo("开爪失败") 

    if success:
        rospy.loginfo("Reaching Cartesian Pose...")
        target_pose = moveiter.get_cartesian_pose()
        target_pose.position.x =grasp_msg.position_x
        target_pose.position.y =grasp_msg.position_y 
        target_pose.position.z =grasp_msg.position_z
        target_pose.orientation.x =grasp_msg.orientation_x
        target_pose.orientation.y =grasp_msg.orientation_y
        target_pose.orientation.z =grasp_msg.orientation_z
        target_pose.orientation.w =grasp_msg.orientation_w
        success &= moveiter.reach_cartesian_pose(pose=target_pose, tolerance=0.01, constraints=None)
        if success:    
            rospy.loginfo("执行成功") 
        else :
            rospy.loginfo("执行失败") 
    else:
        rospy.loginfo ("moveiter 初始化失败")
    
    if moveiter.is_gripper_present and success:
        rospy.loginfo("closing the gripper...")
        success &= moveiter.reach_gripper_position(1)
        if success:    
            rospy.loginfo("合爪成功") 
        else :
            rospy.loginfo("合爪失败") 
    
    # if success:
    rospy.loginfo("正在返回初始状态...")
    success &= moveiter.reach_cartesian_pose(pose=init_pose, tolerance=0.01, constraints=None)
    if success:    
        rospy.loginfo("返回成功") 
    else :
        rospy.loginfo("返回失败") 


def grasp_moveit():
    global init_pose
    # make a video_object and init the video object
    global moveiter
    
    # ROS节点初始化
    rospy.init_node('grasp_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/grasp/plan", Grasp, callback)
    moveiter = ExampleMoveItTrajectories()
    init_pose = moveiter.get_cartesian_pose()
    rospy.spin()



if __name__ == '__main__':
    grasp_moveit()
