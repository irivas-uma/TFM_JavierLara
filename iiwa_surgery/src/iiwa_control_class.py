#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from pytransform3d.pytransform3d.rotations import matrix_from_axis_angle 
from pytransform3d.pytransform3d.rotations import axis_angle_from_matrix
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition


class iiwa_surgery:
    def __init__(self, simulation_mode=True):
        # Variable miembro para configurar el modo de trabajo (simulación o robot real)
        self.simulation_mode = simulation_mode
        # Variables miembro para los datos de la herramienta
        self.tool_length = 0.0
        self.tool_orientation = [0.0, 0.0, 0.0]
        # Variable miembro para el punto de fulcro
        self.fulcrum_fi = 0.0
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
        
    def set_fulcrum_fi(self, fulcrum_fi):
        # Método para configurar el punto de fulcro
        if 0 <= fulcrum_fi <= 1:
            # Variable miembro para el punto de fulcro
            self.fulcrum_fi = fulcrum_fi
        else:
            rospy.logwarn("El punto de fulcro debe ser un valor decimal entre 0 y 1.")
        
    def set_robot_ip(self, robot_ip):
        # Variables miembro con el valor de la IP del robot
        self.robot_ip = robot_ip

    def set_work_mode(self, work_mode):
        if work_mode in ["free", "pivot"]:
            self.work_mode = work_mode
        else:
            rospy.logwarn("Modo de trabajo no válido.")

        
    def move_joint(self, joint_config):
        joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        joint_msg = JointPosition()
        joint_msg = joint_config
        joint_pub.publish(joint_msg)

        
    def move_cartesian(self, pose):
        cartesian_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        cartesian_msg = PoseStamped()

        # Obtener la posición y orientación del mensaje de pose
        position = pose.pose.position
        orientation = pose.pose.orientation

        # Ajustar la posición en función de la longitud de la herramienta
        position.z -= self.tool_length

        # Actualizar la orientación en función de la orientación de la herramienta
        orientation.x -= self.tool_orientation[0]
        orientation.y -= self.tool_orientation[1]
        orientation.z -= self.tool_orientation[2]

        # Actualizar el mensaje de pose con los valores ajustados
        cartesian_msg.header = pose.header
        cartesian_msg.pose.position = position
        cartesian_msg.pose.orientation = orientation

        # Publicar el mensaje en el topic /iiwa/command/CartesianPose
        cartesian_pub.publish(cartesian_msg)

       
    def move_cartesian_fulcrum(self, pose, increment_vector):

        cartesian_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        cartesian_msg = PoseStamped()
               
        #PRIMERO TENGO QUE PASAR DE TCP A EF Y  LUEGO HACER LAS OPERACIONES CORRESPONDIENTES PARA HACER EL MOVIMIENTO DEL PUNTO DE FULCRO
        
        # Otener la posición y orientación del mensaje de pose
        xi = pose.pose.position.x
        yi = pose.pose.position.y
        zi = pose.pose.position.z
        ox = pose.pose.orientation.x
        oy = pose.pose.orientation.y
        oz = pose.pose.orientation.z

        # Realizar los cálculos necesarios para el movimiento cartesiano alrededor del punto de fulcro
        Dt = self.tool_length

        # Asignar los valores de increment_vector a Ph1, Ph2 y Ph3. Éstos serán los incrementos para mover la herramienta
        Ph1 = increment_vector[0]
        Ph2 = increment_vector[1]
        Ph3 = increment_vector[2]


        #-------------Vamos a transformar la orientacion recibida del robot--------------------
        #-------------en otra forma de orientacion que podemos manejar mas adelante------------
           
        #Primero calculamos el ANGULO y el EJE del vector de orientacion que recibimos 
        angle = math.sqrt(ox*ox + oy*oy + oz*oz)
        axis = [ox/angle, oy/angle, oz/angle]

        #Creamos una matriz de orientacion a partir del angulo y el eje anteriores
        a = np.hstack((axis, (angle,)))
        Rm = matrix_from_axis_angle(a)

        #Obtenemos el Eje Z de nuestra matriz, que es el que necesitamos para los calculos
        z = Rm[2]

        #Guardamos la posicion del robot en la variable P
        P = [xi,yi,zi]

        #-------------Una vez tenemos posicion y orientacion vamos a calcular la nueva--------------------
        #-------------posicion y orientacion segun el movimiento que queremos para la herramienta------------
            
        #Para la primera vez que realizamos las operaciones vamos a tomar
        #una serie de valores determinados

        if(j == 0):
            Pf = P + self.fulcrum_fi*Dt*z #El punto de fulcro lo calculamos al principio y no cambia
            j = j+1
            print(Pf)

        #La posicion inicial de la herramienta se calcula a partir de los valores de posicion del robot mas la orientacion
        Pt = P + Dt*z

        #Nueva posicion de la punta con el incremento del Phantom
        Ptn = [Pt[0]+Ph1, Pt[1]+Ph2, Pt[2]+Ph3]

        #Nueva direccion en el eje z de la herramienta
        zn = Pf - Ptn

        #Distancia del punto de fulcro a la nueva posicion de la pinza
        Mzn = math.sqrt(zn[0]*zn[0] + zn[1]*zn[1] + zn[2]*zn[2])
        ro = Dt - Mzn

        #Nueva posicion del efector final del robot
        Pn = Pf + ro*zn/Mzn

        # El eje Z ya lo tenemos, pero lo hacemos unitario
        znn = [-zn[0]/Mzn, -zn[1]/Mzn, -zn[2]/Mzn]


        #-------------Para calcular la orientacion vamos a calcular los angulos de Euler--------------------
        #-------------a partir del eje Z de la matriz, el cual ya tenemos, y una vez tengamos---------------
        #-------------los angulos, llamamos a la funcion que nos calcula la matriz de orientacion-----------

        b = math.atan2(math.sqrt(znn[0]**2 + znn[1]**2), znn[2])
        a = 0
        g = math.atan2((znn[1]/math.sin(b)),(-znn[0]/math.sin(b)))

        M = [[math.cos(a)*math.cos(b)*math.cos(g) - math.sin(a)*math.sin(g), -math.cos(a)*math.cos(b)*math.sin(g) - math.sin(a)*math.cos(a), math.cos(a)*math.sin(b)],
            [math.sin(a)*math.cos(b)*math.cos(g) + math.cos(a)*math.sin(g), -math.sin(a)*math.cos(b)*math.sin(g) + math.cos(a)*math.cos(g), math.sin(a)*math.sin(b)],
            [-math.sin(b)*math.cos(g), math.sin(b)*math.sin(g), math.cos(b)]]
            

        #Pasamos la orientacion a axis-angle, que es la que entiende nuestro simulador
        a = axis_angle_from_matrix(M)
        orientation = [a[0]*a[3], a[1]*a[3], a[2]*a[3]]

        r = Pn-Ptn
        print("Incremento", [Ph1, Ph2, Ph3])
        print(" ")
        print('Nueva posicion efector Pn', Pn)
        print('Nueva posicion herramienta Ptn', Ptn)
        print(' ')
        print("Comprobaciones")
        print(' ')
        print('Actual posicion herramienta Pt', Pt) 
        print('Longitud herramienta Dt', math.sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]))
        print(' ')
        print(' ')
        
            
        # Obtener la posición y orientación del mensaje de pose
        cartesian_msg.header = pose.header
        cartesian_msg.pose.position.x = Pn[0]
        cartesian_msg.pose.position.y = Pn[1]
        cartesian_msg.pose.position.z = Pn[2]
        cartesian_msg.pose.orientation.x = orientation[0]
        cartesian_msg.pose.orientation.y = orientation[1]
        cartesian_msg.pose.orientation.z = orientation[2]


        #Publicamos cartesian_msg
        cartesian_pub.publish(cartesian_msg)

        

