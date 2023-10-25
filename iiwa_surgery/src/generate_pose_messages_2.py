#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
import time
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

class GeneratePoseMessages:
    def __init__(self, initial_position, initial_orientation):
        rospy.init_node('generate_pose_messages_node')
        self.pose_pub = rospy.Publisher('iiwa_surgery/command/pose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(0.33)  # Publica un mensaje cada 3 segundos

        # Variable de instancia para rastrear si se ha enviado el primer mensaje
        self.first_message_sent = False
        
        # Variables de instancia para la posición y orientación inicial proporcionadas
        self.initial_position = initial_position
        self.last_position = initial_position
        self.initial_orientation = initial_orientation

        # Espera un corto período para que ROS establezca la conexión con el robot
        rospy.sleep(1)

        # Publica la posición inicial proporcionada al inicio
        initial_pose = self.generate_pose_message(self.last_position, self.initial_orientation)
        self.pose_pub.publish(initial_pose)
        self.first_message_sent = True
        
    def generate_random_pose(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        #pose.header.frame_id = "base_link"  # Posibilidad de ajustar el frame_id 
        
        orientation = self.initial_orientation  # Mantiene la misma orientación en mensajes posteriores
        
        # Elige aleatoriamente uno de los tres ejes para cambiar
        axis_to_change = random.choice(["x", "y", "z"])
        max_position_increment = 0.04  # Ajusta la medida máxima según tus necesidades
        
        if axis_to_change == "x":
            self.last_position.x = random.uniform(-max_position_increment, max_position_increment) + self.initial_position.x
        elif axis_to_change == "y":
            self.last_position.y = random.uniform(-max_position_increment, max_position_increment) + self.initial_position.y
        else:
            self.last_position.z = random.uniform(-max_position_increment, max_position_increment) + self.initial_position.z
        
        pose.pose.position = self.last_position
        pose.pose.orientation = orientation
        
        return pose

    def generate_pose_message(self, position, orientation):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = "base_link" # Posibilidad de ajustar el frame_id 

        pose.pose.position = position
        pose.pose.orientation = orientation

        return pose

    def run(self):
        while not rospy.is_shutdown():
            # Genera un nuevo mensaje de posición aleatoria con el mismo eje cambiado
            pose = self.generate_random_pose()
            self.pose_pub.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Proporciona la posición y orientación inicial deseada aquí
        initial_position = PoseStamped()
        #initial_position.header.frame_id = "base_link"  # Posibilidad de ajustar el frame_id 
        initial_position.pose.position.x = 0.63  # Modificar a la posición inicial deseada
        initial_position.pose.position.y = 0.0  # Modificar esto según la posición inicial deseada
        initial_position.pose.position.z = 0.4  # Modificar esto según la posición inicial deseada

        initial_orientation = Quaternion()
        initial_orientation.x = 0.0  # Modifica esto a la orientación inicial deseada
        initial_orientation.y = 0.7  # Modifica esto a la orientación inicial deseada
        initial_orientation.z = -0.12  # Modifica esto a la orientación inicial deseada
        initial_orientation.w = 0.1  # Modifica esto a la orientación inicial deseada

        node = GeneratePoseMessages(initial_position.pose.position, initial_orientation)
        node.run()
    except rospy.ROSInterruptException:
        pass
