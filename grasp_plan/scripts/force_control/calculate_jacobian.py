#!/usr/bin/env python
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
from pathlib import Path
from grasp_plan.msg import Force

urdf_path = Path.cwd().joinpath("src/grasp_plan/scripts/force_control/GEN3_URDF_V12.urdf")

robot = URDF.from_xml_file(str(urdf_path)) # get a tree
tree = kdl_tree_from_urdf_model(robot)
kdl_kin = KDLKinematics(robot, "base_link", "EndEffector_Link", tree)
import tf
# q = [0,0,0,0,0,0,0]
# pose = kdl_kin.forward(q) # forward kinematics (returns homogeneous 4x4 matrix)
# print(pose)


global max_torqe
max_torqe = 0
def callback(jointState_msg):
    global max_torqe,publisher,force_msg
    q =np.array(jointState_msg.position[:7])
    torque_joint =np.array(jointState_msg.effort[:7])
    # print(q,torque_joint)
    # rospy.loginfo( "I heard %s %s", data1.data,data2.data)
    J = kdl_kin.jacobian(q)
    # print(jointState_msg.effort[:7])
    force_endeffector = np.linalg.inv(np.dot(J,J.T)).dot(J).dot(torque_joint.T)
    # print(force_endeffector)
    f_x = force_endeffector[0,0]
    f_y = force_endeffector[0,1]
    f_z = force_endeffector[0,2]

    pose = kdl_kin.forward(q)
    # print(pose)
    tcp_z_axis  = pose[:3,2]
    # force_result = np.dot(force_endeffector[0,:3],tcp_z_axis)
    f_total=np.sqrt(np.square(f_x)+np.square(f_y)+np.square(f_z))
    t = torque_joint
    t_total = np.sqrt(np.square(t[0])+np.square(t[1])+np.square(t[2])+np.square(t[3])+np.square(t[4])+np.square(t[5])+np.square(t[6]))
    # print(force_endeffector[0,3:])
    if t_total>max_torqe:
        max_torqe = t_total 
    print("f_total:",f_total)
    print(t_total)
    print("max_torqe:",max_torqe)#32.3488
    if t_total >35:
        force_msg.collision = True
        # force_msg.t_total = t_total
        # force_msg.f_x =  f_x
        # force_msg.f_y =  f_y
        # force_msg.f_z =  f_z
        publisher.publish(force_msg)
        print("published")
    else:
        force_msg.collision = False
        publisher.publish(force_msg)


    # try:
    #     (trans,rot) = listener.lookupTransform('base_link', 'end_effector_link', rospy.Time(0))
    
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     pass



def force_listener():
    global listener,publisher,force_msg
    rospy.init_node('force_control_node', anonymous=True)
    rospy.Subscriber("/my_gen3/joint_states", JointState, callback)
    listener = tf.TransformListener()
    publisher = rospy.Publisher('/grasp/force', Force, queue_size=1)
    force_msg = Force()
    # t1= message_filters.Subscriber("chatter1", String)
    # t2 =message_filters.Subscriber("chatter2", String)
    # ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    # ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()

if __name__ == '__main__':
    force_listener()
