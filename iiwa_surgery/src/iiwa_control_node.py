#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition, JointVelocity, JointTorque
from iiwa_control_class import iiwa_surgery
from sensor_msgs.msg import JointState

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
        
        self.joint_pub = rospy.Publisher('iiwa_surgery/output/joints', JointState, queue_size=10)
        self.ef_pose_pub = rospy.Publisher('iiwa_surgery/output/ef_pose', PoseStamped, queue_size=10)
        self.tcp_pose_pub = rospy.Publisher('iiwa_surgery/output/tcp_pose', PoseStamped, queue_size=10)

        if self.simulation_mode:
            self.joint_state_sub = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback)
        else:
            self.joint_position_sub = rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, self.joint_position_callback)
            self.joint_velocity_sub = rospy.Subscriber('/iiwa/state/JointVelocity', JointVelocity, self.joint_velocity_callback)
            self.joint_effort_sub = rospy.Subscriber('/iiwa/state/JointTorque', JointTorque, self.joint_effort_callback)

        self.combined_joint_state = JointState()
        self.combined_joint_state.header = rospy.Header()
        self.combined_joint_state.name = [
         "iiwa_joint_1",
         "iiwa_joint_2",
         "iiwa_joint_3",
         "iiwa_joint_4",
         "iiwa_joint_5",
         "iiwa_joint_6",
         "iiwa_joint_7"
        ]
        self.combined_joint_state.position = []
        self.combined_joint_state.velocity = []
        self.combined_joint_state.effort = []

        self.cartesian_pose_sub = rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, self.cartesian_pose_callback)
        
        
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

    def joint_state_callback(self, msg):
        self.joint_pub.publish(msg)    
    
    def joint_position_callback(self, msg):
        self.combined_joint_state.header = msg.header
        self.combined_joint_state.position = msg.position
        self.combine_joint_states()
        
    def joint_velocity_callback(self, msg):
        self.combined_joint_state.velocity = msg.velocity
        self.combine_joint_states()
        
    def joint_effort_callback(self, msg):
        self.combined_joint_state.effort = msg.torque
        self.combine_joint_states()

    def combine_joint_states(self):        
        self.joint_pub.publish(self.combined_joint_state)

    def cartesian_pose_callback(self, msg):
        # Publicar en iiwa_surgery/output/ef_pose
        self.ef_pose_pub.publish(msg)
        
        # Publicar en iiwa_surgery/output/tcp_pose
        tcp_pose = PoseStamped()
               
        # Obtener la posición y orientación del mensaje de msg
        position = msg.pose.position
        orientation = msg.pose.orientation

        # Ajustar la posición en función de la longitud de la herramienta
        position.z += self.tool_length

        # Actualizar la orientación en función de la orientación de la herramienta
        orientation.x += self.tool_orientation[0]
        orientation.y += self.tool_orientation[1]
        orientation.z += self.tool_orientation[2]

        # Actualizar el mensaje de tcp_pose con los valores ajustados
        tcp_pose.header = msg.header
        tcp_pose.pose.position = position
        tcp_pose.pose.orientation = orientation

        # Publicar el mensaje en el topic iiwa_surgery/output/tcp_pose
        self.tcp_pose_pub.publish(tcp_pose)


    def run(self):
        rate = rospy.Rate(10)  # Frecuencia de bucle de ejecución
        
        while not rospy.is_shutdown():
            # Realizar cualquier procesamiento adicional si es necesario
            
            rate.sleep()

if __name__ == '__main__':
    node = IiwaSurgeryNode()
    node.run()