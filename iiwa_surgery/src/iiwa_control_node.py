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

        # Obtener los valores de los parámetros del sistema de parámetros de ROS
        self.simulation_mode = rospy.get_param("simulation_mode", True)
        self.tool_length = rospy.get_param("tool_length", 0.0)
        self.tool_orientation = rospy.get_param("tool_orientation", [0.0, 0.0, 0.0])
        self.fulcrum_point = rospy.get_param("fulcrum_point", 0.0)
        self.robot_ip = rospy.get_param("robot_ip", "")
        self.work_mode = rospy.get_param("work_mode", "free")

        # Pasar los valores de los parámetros a la instancia de la clase iiwa_surgery
        self.iiwa.simulation_mode = self.simulation_mode
        self.iiwa.set_tool_data(self.tool_length, self.tool_orientation)
        self.iiwa.set_fulcrum_point(self.fulcrum_point)
        self.iiwa.set_robot_ip(self.robot_ip)
        self.iiwa.set_work_mode(self.work_mode)
        
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
        if self.iiwa.work_mode == "free":
            pose = PoseStamped()
            pose = msg
            self.iiwa.move_cartesian(pose)
        elif self.iiwa.work_mode == "pivot":
            pose = PoseStamped()
            pose = msg
            self.iiwa.move_cartesian_fulcrum(pose)
        else:
            rospy.logwarn("Modo de trabajo no válido.")        
        
    def run(self):
        rate = rospy.Rate(10)  # Frecuencia de bucle de ejecución
        
        while not rospy.is_shutdown():
            # Realizar cualquier procesamiento adicional si es necesario
            
            rate.sleep()

if __name__ == '__main__':
    node = IiwaSurgeryNode()
    node.run()