#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped 
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.msg import JointPosition
from iiwa_control_class import iiwa_surgery
from sensor_msgs.msg import JointState
from conversions import matrix_from_quaternion

class IiwaSurgeryNode:
    """!
    Clase que define el nodo ROS para el control del robot quirúrgico iiwa.
    """
    def __init__(self):
        """!
        Constructor de la clase.
        @protected
        """
        rospy.init_node('iiwa_surgery_node')

        ## @protected
        # Es una instancia de la clase iiwa_surgery que se utiliza para controlar el robot quirúrgico iiwa.
        self.iiwa = iiwa_surgery() 

        ## @protected
        # Variable que se utiliza para almacenar la primera posición recibida en el callback pose_command_callback.
        self.first_position = None

        # Obtener los valores de los parametros del sistema de parametros de ROS

        ## @protected
        # Indica si el robot está en modo de simulación o no, siendo True para simulación y False para el robot real.
        self.simulation_mode = rospy.get_param("simulation_mode", True) 

        ## @protected
        # Almacena la longitud de la herramienta utilizada en el robot quirúrgico.
        self.tool_length = rospy.get_param("tool_length", 0.0) 

        ## @protected
        # Almacena la orientación de la herramienta en radianes.
        self.tool_orientation = rospy.get_param("tool_orientation", [0.0, 0.0, 0.0]) 

        ## @protected
        # Representa el ángulo de fulcro en grados utilizado en el control del robot quirúrgico.
        self.fulcrum_fi = rospy.get_param("fulcrum_fi", 0.0) 

        ## @protected
        # Almacena la dirección IP del robot quirúrgico iiwa.
        self.robot_ip = rospy.get_param("robot_ip", "") 

        ## @protected
        # Indica el modo de trabajo predeterminado del robot quirúrgico ("free" para movimiento libre y "pivot" para movimiento alrededor de punto de fulcro).
        self.work_mode = rospy.get_param("work_mode", "free") 

        # Pasar los valores de los parametros a la instancia de la clase iiwa_surgery
        self.iiwa.simulation_mode = self.simulation_mode
        self.iiwa.tool_length = self.tool_length
        self.iiwa.set_tool_data(self.tool_length, self.tool_orientation)
        self.iiwa.set_fulcrum_fi(self.fulcrum_fi)
        self.iiwa.set_robot_ip(self.robot_ip)
        self.iiwa.set_work_mode(self.work_mode)

        # Configurar suscriptores y publicadores de ROS

        ## @protected
        # Suscriptor ROS utilizado para recibir comandos de posición articular.
        self.joint_sub = rospy.Subscriber('iiwa_surgery/command/joints', JointPosition, self.joint_command_callback)
        
        ## @protected
        # Suscriptor ROS utilizado para recibir comandos de posición cartesiana.
        self.pose_sub = rospy.Subscriber('iiwa_surgery/command/pose', PoseStamped, self.pose_command_callback)
        
        ## @protected
        # Publicador ROS utilizado para enviar mensajes de posición articular.
        self.joint_pub = rospy.Publisher('iiwa_surgery/output/joints', JointPosition, queue_size=10)
        
        ## @protected
        # Publicador ROS utilizado para enviar estados de posición final del efector (EF) del robot.
        self.ef_pose_pub = rospy.Publisher('iiwa_surgery/output/ef_pose', PoseStamped, queue_size=10)
        
        ## @protected
        # Publicador ROS utilizado para enviar estados de posición del Tool Center Point (TCP).
        self.tcp_pose_pub = rospy.Publisher('iiwa_surgery/output/tcp_pose', PoseStamped, queue_size=10)

        if self.simulation_mode:
            ## @protected
            # Suscriptor ROS utilizado para recibir estados de posición articular.
            self.joint_state_sub = rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback)
            ## @protected
            # Suscriptor ROS utilizado para recibir estados de posición cartesiana.
            self.cartesian_pose_sub = rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, self.cartesian_pose_callback) 
        else:
            ## @protected
            # Suscriptor ROS utilizado para recibir estados de posición articular.
            self.joint_position_sub = rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, self.joint_position_callback)
            self.cartesian_pose_sub = rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, self.cartesian_pose_callback) 

        
        
    def joint_command_callback(self, msg):
        """!
        Callback para recibir comandos de posición articular y posteriormente enviarlos al método move_joint de la clase iiwa_surgery.
        @param msg: Mensaje de posición articular (JointPosition).
        @protected
        """
        # Mover el robot a una posicion articular especifica
        joint_config = JointPosition()
        joint_config = msg
        # Enviar mensaje de posicion articular al metodo
        self.iiwa.move_joint(joint_config)       
        
    def pose_command_callback(self, msg):
        """!
        Callback para recibir comandos de posición cartesiana y posteriormente enviarlos a los métodos move_cartesian o move_cartesian_fulcrum de la clase iiwa_surgey dependiendo si el movimiento es libre o de pivoteo.
        @param msg: Mensaje de posición cartesiana (PoseStamped).
        @protected
        """
        if self.work_mode == "free":
            # Mover el robot a una posicion cartesiana especifica en modo "free"
            pose = PoseStamped()
            pose = msg
            self.iiwa.move_cartesian(pose)
        elif self.work_mode == "pivot":
            # Mover el robot en modo "pivot" alrededor de un punto de fulcro
            pose = PoseStamped()
            pose = msg

            # Obtener la posicion x, y, z del mensaje de entrada
            position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

            if self.first_position is None:
                # Primer mensaje recibido, enviar directamente la posicion al metodo
                self.first_position = position
                increment_vector = np.zeros(3)  # Crear un vector de ceros
                j = 0
                self.iiwa.move_cartesian_fulcrum(pose, increment_vector, j)
            else:
                # Calcular vector de incrementos 
                increment_vector = position - self.first_position
                j = 1
                # Enviar vector de incrementos y pose al metodo
                self.iiwa.move_cartesian_fulcrum(pose, increment_vector, j)
        else:
            rospy.logwarn("Modo de trabajo no válido.")  

    def joint_state_callback(self, msg):
        """!
        Callback para recibir estados de posición articular y posteriormente publicarlos en el topic iiwa_surgery/output/joints.
        @param msg: Mensaje de estado de posición articular (JointState).
        @protected
        """        
        joint_position_msg = JointPosition()
        joint_position_msg.header = msg.header
        
        # Asignar los valores de position a los campos a1 a a7
        joint_position_msg.position.a1 = msg.position[0]
        joint_position_msg.position.a2 = msg.position[1]
        joint_position_msg.position.a3 = msg.position[2]
        joint_position_msg.position.a4 = msg.position[3]
        joint_position_msg.position.a5 = msg.position[4]
        joint_position_msg.position.a6 = msg.position[5]
        joint_position_msg.position.a7 = msg.position[6]

        # Publicar el estado de posicion articular 
        self.joint_pub.publish(joint_position_msg)    
    
    def joint_position_callback(self, msg):
        """!
        Callback para recibir estados de posición articular y posteriormente publicarlos en el topic iiwa_surgery/output/joints.
        @param msg: Mensaje de estado de posición articular (JointPosition).
        @protected
        """        
        # Publicar el estado de posicion articular recibido
        self.joint_pub.publish(msg)

    def cartesian_pose_callback(self, msg):
        """!
        Callback para recibir estados de posición cartesianay posteriormente publicarlos en el topic iiwa_surgery/output/ef_pose y tras trasnformación en iiwa_surgery/output/tcp_pose.
        @param msg: Mensaje de estado de posición cartesiana (PoseStamped).
        @protected
        """
        cartesian_pose = PoseStamped()

        if self.simulation_mode:
            # Cuando se trabaja con el simulador, usar el mensaje directamente
            cartesian_pose = msg
        else:
            # Cuando se trabaja con el robot real, mapear a geometry_msgs.PoseStamped
            cartesian_pose.header = msg.poseStamped.header
            cartesian_pose.pose.position = msg.poseStamped.pose.position
            cartesian_pose.pose.orientation = msg.poseStamped.pose.orientation
        

        # Publicar la información recibida
        self.ef_pose_pub.publish(cartesian_pose)

        # Calcular la posicion del TCP a partir del EF

        # Obtener la posicion y orientacion del mensaje
        position = cartesian_pose.pose.position
        orientation = cartesian_pose.pose.orientation

        # Reordenar el cuaternio en el vector q (w, x, y, z)
        q = [orientation.w, orientation.x, orientation.y, orientation.z]
         
        # Crear una matriz de rotacion a partir del cuaternio recibido 
        Rm = matrix_from_quaternion(q)

        # Calcular el vector de direccion relativo al EF
        direction = np.array([0,0,self.tool_length])
        direction_transformed = np.dot(Rm, direction)

        # Guardar la posicion del EF del robot en la variable Pef
        Pef = np.array([position.x,position.y,position.z])

        # Calcular la posicion del TCP
        Ptcp = Pef + direction_transformed

        # Actualizar valores de posicion del TCP
        position.x = Ptcp[0]
        position.y = Ptcp[1]
        position.z = Ptcp[2]

        # Crear un mensaje del tipo PoseStamped 
        tcp_pose = PoseStamped()

        # Actualizar el mensaje de tcp_pose con los valores ajustados
        tcp_pose.header = cartesian_pose.header
        tcp_pose.pose.position = position
        tcp_pose.pose.orientation = orientation # La herramienta tendra la misma orientacion que el EF

        # Publicar el mensaje 
        self.tcp_pose_pub.publish(tcp_pose)


    def run(self):
        """!
        Ejecuta el bucle principal mientras el nodo está en funcionamiento.
        @protected
        """
        rate = rospy.Rate(10)  # Frecuencia de bucle de ejecucion
        
        while not rospy.is_shutdown():
            # Realizar cualquier procesamiento adicional si es necesario
            
            rate.sleep()

if __name__ == '__main__':
    node = IiwaSurgeryNode()
    node.run()
    