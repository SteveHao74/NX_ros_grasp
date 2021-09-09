#!/usr/bin/env python3
#encoding: utf-8
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from grasp_plan.msg import Force
from grasp_plan.msg import Plot

# q = [0,0,0,0,0,0,0]
# pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 matrix)
# print(pose)

if __name__ == '__main__':
    global listener,publisher,force_msg
    rospy.init_node('temp_test', anonymous=True)
    # rospy.Subscriber("/my_gen3/joint_states", JointState, callback)
    # listener = tf.TransformListener()
    publisher = rospy.Publisher('/my_gen3/temp', Force, queue_size=1)
    plot_publisher = rospy.Publisher('/grasp/plot', Plot, queue_size=1)
    force_msg = Force()
    plot_msg = Plot()
    plot_msg.p0 = [100,600]
    plot_msg.p1 = [100,50]

	#设置循环的频率
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        excute_flag = input("automatic grasp now (y/n):")
        if excute_flag == "y":
            force_msg.collision = True
            publisher.publish(force_msg)
            plot_publisher.publish(plot_msg)
            print("published")
        else:
            print("none")

# 按照循环频率延时
rate.sleep()