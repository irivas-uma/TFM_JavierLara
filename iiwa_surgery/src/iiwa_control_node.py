#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition
from iiwa_control_class import iiwa_surgery
from sensor_msgs.msg import JointState
from conversions import axis_angle_from_quaternion
from conversions import matrix_from_quaternion
from conversions import quaternion_from_matrix
from conversions import euler_from_quaternion 

class IiwaSurgeryNode:
    def __init__(self):
        rospy.init_node('iiwa_surgery_node')
        self.iiwa = iiwa_surgery()
        self.last_position = None

        # Obtener los valores de los parámetros del sistema de parámetros de ROS
        self.simulation_mode = rospy.get_param("simulation_mode", True)
        self.tool_length = rospy.get_param("tool_length", 0.0)
        self.tool_orientation = rospy.get_param("tool_orientation", [0.0, 0.0, 0.0])
        self.fulcrum_fi = rospy.get_param("fulcrum_fi", 0.0)
        self.robot_ip = rospy.get_param("robot_ip", "")
        self.work_mode = rospy.get_param("work_mode", "free")

        # Pasar los valores de los parámetros a la instancia de la clase iiwa_surgery
        self.iiwa.simulation_mode = self.simulation_mode
        self.iiwa.set_tool_data(self.tool_length, self.tool_orientation)
        self.iiwa.set_fulcrum_fi(self.fulcrum_fi)
        self.iiwa.set_robot_ip(self.robot_ip)
        self.iiwa.set_work_mode(self.work_mode)
        
        self.joint_sub = rospy.Subscriber('iiwa_surgery/command/joints', JointPosition, self.joint_command_callback)
        self.pose_sub = rospy.Subscriber('iiwa_surgery/command/pose', PoseStamped, self.pose_command_callback)
        
        self.joint_pub = rospy.Publisher('iiwa_surgery/output/joints', JointPosition, queue_size=10)
        self.ef_pose_pub = rospy.Publisher('iiwa_surgery/output/ef_pose', PoseStamped, queue_size=10)
        self.tcp_pose_pub = rospy.Publisher('iiwa_surgery/output/tcp_pose', PoseStamped, queue_size=10)

        if self.simulation_mode:
            self.joint_state_sub = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback)
        else:
            self.joint_position_sub = rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, self.joint_position_callback)

        self.cartesian_pose_sub = rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, self.cartesian_pose_callback)       
        
        
    def joint_command_callback(self, msg):
        joint_config = JointPosition()
        joint_config = msg
        self.iiwa.move_joint(joint_config)       
        
    def pose_command_callback(self, msg):
        if self.work_mode == "free":
            pose = PoseStamped()
            pose = msg
            self.iiwa.move_cartesian(pose)
        elif self.work_mode == "pivot":
            pose = PoseStamped()
            pose = msg

            # Obtener la posición x, y, z del mensaje de entrada
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

            if self.last_position is None:
                # Es el primer mensaje recibido, enviar directamente la posición al método
                self.last_position = position
                increment_vector = np.zeros(3)  # Crear un vector de ceros
                self.iiwa.move_cartesian_fulcrum(pose, increment_vector)
            else:
                # Calcular el vector de incrementos como la diferencia entre position y self.last_position
                increment_vector = position - self.last_position
                self.last_position = position

                # Pasar el vector de incrementos y la pose al método
                self.iiwa.move_cartesian_fulcrum(pose, increment_vector)
        else:
            rospy.logwarn("Modo de trabajo no válido.")  

    def joint_state_callback(self, msg):
        joint_position_msg = JointPosition()
        joint_position_msg.header = msg.header
        joint_position_msg.position = msg.position
        self.joint_pub.publish(joint_position_msg)    
    
    def joint_position_callback(self, msg):
        self.joint_pub.publish(msg)

    def cartesian_pose_callback(self, msg):
        # Publicamos en iiwa_surgery/output/ef_pose la información recibida del EF en msg
        self.ef_pose_pub.publish(msg)

        # Para obtener el TCP a partir del EF (suponiendo que la herramienta sea un palo) hacemos:

        # Obtener la posición y orientación del mensaje de msg
        position = msg.pose.position
        orientation = msg.pose.orientation

        # Reordenar el cuaternio en el vector q (w, x, y, z)
        q = [orientation.w, orientation.x, orientation.y, orientation.z]
         
        # Creamos una matriz de rotación a partir del quaternio recibido en msg
        Rm = matrix_from_quaternion(q)

        # Obtenemos el Eje Z de nuestra matriz, que es el que necesitamos para los calculos
        z = Rm[2]

        # Guardamos la posicion del robot en la variable P
        P = [position.x,position.y,position.z]

        # Una vez tenemos posición y orientación vamos a calcular la posición del TCP
        
        # La posición de la herramienta se calcula a partir de los valores de posición del robot mas la orientación
        Pt = P + self.tool_length*z

        # Actualizar la orientación en función de la orientación de la herramienta
        position.x = Pt[0]
        position.y = Pt[1]
        position.z = Pt[2]

        # Creamos un mensaje del tipo PoseStamped para publicar en iiwa_surgery/output/tcp_pose 
        tcp_pose = PoseStamped()

        # Actualizar el mensaje de tcp_pose con los valores ajustados
        tcp_pose.header = msg.header
        tcp_pose.pose.position = position
        tcp_pose.pose.orientation = orientation #Como la herramienta es un palo, tendrá la misma orientación que el EF

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