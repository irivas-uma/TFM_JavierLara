#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition
from std_msgs.msg import Float64MultiArray, Float64


class iiwa_surgery:
    def __init__(self, simulation_mode=True):
        # Variable miembro para configurar el modo de trabajo (simulación o robot real)
        self.simulation_mode = simulation_mode
        # Variables miembro para los datos de la herramienta
        self.tool_length = 0.0
        self.tool_orientation = [0.0, 0.0, 0.0]
        # Variable miembro para el punto de fulcro
        self.fulcrum_point = 0.0
        # Variables miembro con el valor de la IP del robot
        self.robot_ip = ""
        # Variables miembro con el modo de trabajo del robot
        self.work_mode = "free"  # Valor predeterminado

        # Otras variables miembro de configuración del robot, si es necesario
        
    def set_tool_data(self, tool_length, tool_orientation):
        # Método para configurar los datos de la herramienta
        # Variables miembro para los datos de la herramienta
        self.tool_length = tool_length
        self.tool_orientation = tool_orientation
        
    def set_fulcrum_point(self, fulcrum_point):
        # Método para configurar el punto de fulcro
        if 0 <= fulcrum_point <= 1:
            # Variable miembro para el punto de fulcro
            self.fulcrum_point = fulcrum_point
        else:
            rospy.logwarn("El punto de fulcro debe ser un valor decimal entre 0 y 1.")
        
    def set_robot_ip(self, robot_ip):
        # Variables miembro con el valor de la IP del robot
        self.robot_ip = robot_ip

    def set_work_mode(self, work_mode):
        # Variables miembro con el modo de trabajo del robot ("pivot" o "free")
        self.work_mode = work_mode
        
    def move_joint(self, joint_config):
        joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        joint_msg = JointPosition()
        joint_msg = joint_config
        joint_pub.publish(joint_msg)

        
    def move_cartesian(self, pose):
        cartesian_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        cartesian_msg = PoseStamped()
        cartesian_msg = pose
        cartesian_pub.publish(cartesian_msg)

    
    def move_cartesian_fulcrum(self, pose):
    # Implementar el movimiento cartesiano alrededor del punto de fulcro
        pass

