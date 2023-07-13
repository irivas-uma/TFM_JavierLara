#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition
from iiwa_control_class import iiwa_surgery

class IiwaSurgeryNode:
    def __init__(self):
        rospy.init_node('iiwa_surgery_node')
        self.iiwa = iiwa_surgery()
        
        self.joint_sub = rospy.Subscriber('iiwa_surgery/command/joints', JointPosition, self.joint_command_callback)
        self.pose_sub = rospy.Subscriber('iiwa_surgery/command/pose', PoseStamped, self.pose_command_callback)
        
        self.joint_pub = rospy.Publisher('iiwa_surgery/output/joints', JointPosition, queue_size=10)
        self.ef_pose_pub = rospy.Publisher('iiwa_surgery/output/ef_pose', PoseStamped, queue_size=10)
        self.tcp_pose_pub = rospy.Publisher('iiwa_surgery/output/tcp_pose', PoseStamped, queue_size=10)
        
    def joint_command_callback(self, msg):
        joint_config = JointPosition()
        joint_config = msg
        self.iiwa.move_joint(joint_config)       
        
    def pose_command_callback(self, msg):
        pose = PoseStamped()
        pose = msg
        self.iiwa.move_cartesian(pose)        
        
    def run(self):
        rate = rospy.Rate(10)  # Frecuencia de bucle de ejecución
        
        while not rospy.is_shutdown():
            # Realizar cualquier procesamiento adicional si es necesario
            
            rate.sleep()

if __name__ == '__main__':
    node = IiwaSurgeryNode()
    node.run()