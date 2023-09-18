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
        self.initial_orientation = initial_orientation

        # Espera un corto período para que ROS establezca la conexión con el robot
        rospy.sleep(1)

        # Publica la posición inicial proporcionada al inicio
        initial_pose = self.generate_pose_message(self.initial_position, self.initial_orientation)
        self.pose_pub.publish(initial_pose)
        self.first_message_sent = True
        
    def generate_random_pose(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        #pose.header.frame_id = "base_link"  # Puedes ajustar el frame_id según tu configuración
        
        orientation = self.initial_orientation  # Mantiene la misma orientación en mensajes posteriores
        # Genera una posición aleatoria limitada por max_position_increment
        max_position_increment = 0.04  # Ajusta la medida máxima según tus necesidades
        pose.pose.position.x = random.uniform(-max_position_increment, max_position_increment) + self.initial_position.x
        pose.pose.position.y = random.uniform(-max_position_increment, max_position_increment) + self.initial_position.y
        pose.pose.position.z = random.uniform(-max_position_increment, max_position_increment) + self.initial_position.z
        
        pose.pose.orientation = orientation
        
        return pose

    def generate_pose_message(self, position, orientation):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        # Puedes ajustar el frame_id según tu configuración
        # pose.header.frame_id = "base_link"

        pose.pose.position = position
        pose.pose.orientation = orientation

        return pose

    def run(self):
        while not rospy.is_shutdown():
            # Genera un nuevo mensaje de posición aleatoria con la misma orientación inicial
            pose = self.generate_random_pose()
            self.pose_pub.publish(pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Proporciona la posición y orientación inicial deseada aquí
        initial_position = PoseStamped()
        #initial_position.header.frame_id = "base_link"
        initial_position.pose.position.x = 0.63  # Modifica esto según tu posición inicial
        initial_position.pose.position.y = 0.0  # Modifica esto según tu posición inicial
        initial_position.pose.position.z = 0.2  # Modifica esto según tu posición inicial

        initial_orientation = Quaternion()
        initial_orientation.x = 0.0  # Modifica esto según tu orientación inicial
        initial_orientation.y = 0.7  # Modifica esto según tu orientación inicial
        initial_orientation.z = -0.12  # Modifica esto según tu orientación inicial
        initial_orientation.w = 0.1  # Modifica esto según tu orientación inicial

        node = GeneratePoseMessages(initial_position.pose.position, initial_orientation)
        node.run()
    except rospy.ROSInterruptException:
        pass
