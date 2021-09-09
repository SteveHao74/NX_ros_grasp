#!/usr/bin/env python
#encoding: utf-8
import rospy
# from sensor_msgs.msg import Image
from grasp_plan.msg import Grasp
from example_full_arm_movement   import  ExampleFullArmMovement
from sensor_msgs.msg import JointState
#-0.06,-0.243,0.414 ; 4.8,174.3,169.7
pos_list = [-0.012,-0.368,0.632 ,159.1,-1,4.3]
# pos_list = [-0.06,-0.243,0.414 ,4.8,174.3,169.7]
# pos_list = [0.15,-0.339,0.088,-66.5,178.9,94.5]
def callback(grasp_msg): 
    global init_pose
    success = moveiter.is_init_success
    grasp_msg.translation_speed=0.8#0.8 # 米
    grasp_msg.orientation_speed=60#15 
    rospy.loginfo("getit...")
    # init_pose = moveiter.get_cartesian_pose()
    joint_state = rospy.wait_for_message("/my_gen3/joint_states", JointState, timeout=0.5)
    init_joint_angle = joint_state.position
    if success:
        success &= moveiter.example_clear_faults()
        success &= moveiter.example_subscribe_to_a_robot_notification()
        moveiter.example_send_joint_angles(init_joint_angle)
        rospy.loginfo("初始化正常")

        if moveiter.is_gripper_present:
            rospy.loginfo("Opening the gripper...")
            success &= moveiter.example_send_gripper_command(0)
        else:
            rospy.logwarn("No gripper is present on the arm.")  

        success &= moveiter.example_set_cartesian_reference_frame()

        if success:
            rospy.loginfo("Reaching Cartesian Pose...")
            moveiter.example_send_cartesian_pose(grasp_msg)

        if success and moveiter.is_gripper_present:    
            rospy.loginfo("执行成功") 
            success &= moveiter.example_send_gripper_command(0.9)
        else :
            rospy.loginfo("执行失败") 
        # if success:
        #     success &= moveiter.example_send_init_pose(grasp_msg,pos_list)
        #     # success &= moveiter.example_home_the_robot()
        # if success:
        #     rospy.loginfo("Opening the gripper...")
        #     success &= moveiter.example_send_gripper_command(0.0)
    if not success:
        rospy.logerr("The example encountered an error.")




def grasp_moveit():
    global init_pose
    # make a video_object and init the video object
    global moveiter
    
    # ROS节点初始化
    rospy.init_node('grasp_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/grasp/plan", Grasp, callback)
    moveiter = ExampleFullArmMovement()

    rospy.spin()



if __name__ == '__main__':
    grasp_moveit()
