#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from conversions import axis_angle_from_quaternion
from conversions import matrix_from_quaternion
from conversions import quaternion_from_matrix
from conversions import euler_from_quaternion 
from conversions import quaternion_from_axis_angle
from conversions import axis_angle_from_matrix
from conversions import eulerZYZ
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition
from grafico import actualizar_grafico

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

        self.cartesian_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        self.joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        # ... otros publicadores y suscriptores
       
        
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
        joint_msg = JointPosition()
        joint_msg = joint_config
        self.joint_pub.publish(joint_msg)

        
    def move_cartesian(self, pose):
        cartesian_msg = PoseStamped()

        # Para obtener el EF a partir del TCP (suponiendo que la herramienta sea un palo) hacemos:

        # Obtener la posición y orientación del mensaje de msg
        position = pose.pose.position
        orientation = pose.pose.orientation

        # Reordenar el cuaternio en el vector q (w, x, y, z)
        q = [orientation.w, orientation.x, orientation.y, orientation.z]
         
        # Creamos una matriz de rotación a partir del quaternio recibido en pose
        Rm = matrix_from_quaternion(q)

        # Calculamos el vector de dirección relativo al EF
        direction = np.array([0,0,-self.tool_length])
        direction_transformed = np.dot(Rm, direction)

        # Guardamos la posicion del TCP en la variable Ptcp
        Ptcp = np.array([position.x,position.y,position.z])

        # Una vez tenemos posición y orientación vamos a calcular la posición del EF
        
        # La posición del EF se calcula como:
        Pef = Ptcp + direction_transformed

        # Obtenemos los valores de la posición del EF
        position.x = Pef[0]
        position.y = Pef[1]
        position.z = Pef[2]

        # Actualizar el mensaje de cartesian_msg con los valores ajustados
        cartesian_msg.header = pose.header
        cartesian_msg.pose.position = position
        cartesian_msg.pose.orientation = orientation #Como la herramienta es un palo, tendrá la misma orientación que el TCP
        
        # Publicar el mensaje en el topic /iiwa/command/CartesianPose
        self.cartesian_pub.publish(cartesian_msg)

       
    def move_cartesian_fulcrum(self, pose, increment_vector, j):
        cartesian_msg = PoseStamped() # En este mensaje se almacenará la información del EF 
      
        # Asignar los valores de increment_vector a Ph1, Ph2 y Ph3. Éstos serán los incrementos para mover la herramienta
        Ph1 = increment_vector[0]
        Ph2 = increment_vector[1]
        Ph3 = increment_vector[2]      

        #Para la primera vez que realizamos las operaciones vamos a tomar una serie de valores determinados

        if(j == 0): #Si es la primera posición que recibimos hacemos:

            # Para obtener el EF inicial a partir del TCP inicial (suponiendo que la herramienta sea un palo) hacemos:

            # Obtener la posición y orientación del mensaje de msg
            position = pose.pose.position
            orientation = pose.pose.orientation

            # Reordenar el cuaternio en el vector q (w, x, y, z)
            q = [orientation.w, orientation.x, orientation.y, orientation.z]
         
            # Creamos una matriz de rotación a partir del quaternio recibido en pose
            Rm = matrix_from_quaternion(q)

            # Calculamos el vector de dirección relativo al TCP inicial
            direction = np.array([0,0,-self.tool_length])
            direction_transformed = np.dot(Rm, direction)

            # Guardamos la posicion del TCP inicial en la variable self.Ptcp
            self.Ptcp = np.array([position.x,position.y,position.z])

            # Una vez tenemos posición y orientación del TCP inicial vamos a calcular la posición del EF inicial
        
            # La posición del EF inical se calcula como:
            Pef = self.Ptcp + direction_transformed
  
            # Una vez tenemos posición y orientación del EF inicial vamos a calcular la posición del punto de fulcro
            # Calculamos el vector de dirección relativo al EF 
            direction = np.array([0,0, self.fulcrum_fi*self.tool_length])
            direction_transformed = np.dot(Rm, direction)
        
            # La posición del punto de fulcro se calcula a partir de los valores de posición del EF inicial más el vector de dirección transformado por el valor fulcrum_fi
            self.Pf = Pef + direction_transformed #El punto de fulcro lo calculamos al principio y no cambia
            actualizar_grafico(self.Pf)

            print("Punto de fulcro: ", self.Pf)
            print("Posicion EF inical: ", Pef)
            print("Posicion TCP inicial: ", self.Ptcp)
            print(" ")

            
            # print("Incremento", [Ph1, Ph2, Ph3])
            # print(" ")
            # print(' ')
            # print("Actual posicion herramienta", self.Ptcp) 
            # print("Longitud herramienta", self.tool_length)
            # print(' ')
            # print(' ')

            # Introducimos en cartesian_msg la posición y orientación del EF inicial
            cartesian_msg.header = pose.header
            cartesian_msg.pose.position.x = Pef[0]
            cartesian_msg.pose.position.y = Pef[1]
            cartesian_msg.pose.position.z = Pef[2]
            cartesian_msg.pose.orientation.w = q[0]
            cartesian_msg.pose.orientation.x = q[1]
            cartesian_msg.pose.orientation.y = q[2]
            cartesian_msg.pose.orientation.z = q[3]

        

        if(j == 1): #Si no es la primera posición que recibimos hacemos:
        
            #Nueva posición de la punta de la herramienta con el incremento
            Ptn = [self.Ptcp[0]+Ph1, self.Ptcp[1]+Ph2, self.Ptcp[2]+Ph3]

            #Nueva direccion en el eje z de la herramienta
            zn = self.Pf - Ptn

            #Distancia del punto de fulcro a la nueva posición de la pinza
            Mzn = math.sqrt(zn[0]*zn[0] + zn[1]*zn[1] + zn[2]*zn[2])
            ro = self.tool_length - Mzn

            #Nueva posicion del efector final del robot
            Pn = self.Pf + ro*zn/Mzn

            # El eje Z ya lo tenemos, pero lo hacemos unitario
            znn = [-zn[0]/Mzn, -zn[1]/Mzn, -zn[2]/Mzn]


            #-------------Para calcular la orientacion vamos a calcular los angulos de Euler--------------------
            #-------------a partir del eje Z de la matriz, el cual ya tenemos, y una vez tengamos---------------
            #-------------los angulos, llamamos a la funcion que nos calcula la matriz de orientacion-----------

            b = math.atan2(math.sqrt(znn[0]**2 + znn[1]**2), znn[2])
            a = 0
            g = math.atan2((znn[1]/math.sin(b)),(-znn[0]/math.sin(b)))

            M = eulerZYZ([a,b,g])

            # M = [[math.cos(a)*math.cos(b)*math.cos(g) - math.sin(a)*math.sin(g), -math.cos(a)*math.cos(b)*math.sin(g) - math.sin(a)*math.cos(a), math.cos(a)*math.sin(b)],
            #     [math.sin(a)*math.cos(b)*math.cos(g) + math.cos(a)*math.sin(g), -math.sin(a)*math.cos(b)*math.sin(g) + math.cos(a)*math.cos(g), math.sin(a)*math.sin(b)],
            #     [-math.sin(b)*math.cos(g), math.sin(b)*math.sin(g), math.cos(b)]]
            

            #Pasamos la orientacion a axis-angle 
            a = axis_angle_from_matrix(M)

            #Pasamos la orientacion de axis-angle a cuaternio
            q = quaternion_from_axis_angle(a)
       

            # print("Incremento", [Ph1, Ph2, Ph3])
            # print(" ")
            # print('Nueva posicion efector Pn', Pn)
            # print('Nueva posicion herramienta Ptn', Ptn)
            # print(' ')
            # print("Comprobaciones")
            # print(' ')
            # print("Longitud herramienta Dt", self.tool_length)
            # print(' ')
            # print(' ')
        
            
            # Introducimos en cartesian_msg la posición y orientación del nuevo EF
            cartesian_msg.header = pose.header
            cartesian_msg.pose.position.x = Pn[0]
            cartesian_msg.pose.position.y = Pn[1]
            cartesian_msg.pose.position.z = Pn[2]
            cartesian_msg.pose.orientation.w = q[0]
            cartesian_msg.pose.orientation.x = q[1]
            cartesian_msg.pose.orientation.y = q[2]
            cartesian_msg.pose.orientation.z = q[3]

            #COMPROBACIONES

            #Teniendo las nuevaS posiciONES del EF y del TCP con los incrementos recibidos, vamos a calcular el punto de fulcro para ver si se ha ejecutado correctamente

            # Creamos una matriz de rotación a partir del quaternio que tiene la orientación del nuevo EF
            Rm = matrix_from_quaternion(q)
            #Para ello a partir del nuevo EF calculamos el punto de fulcro con el proceso que ya hemos hecho ya antes
            # Calculamos el vector de dirección relativo al nuevo EF 
            direction = np.array([0,0, self.fulcrum_fi*self.tool_length])
            direction_transformed = np.dot(Rm, direction)
        
            # La posición del punto de fulcro se calcula a partir de los valores de posición del nuevo EF más el vector de dirección transformado por el valor fulcrum_fi
            Pf_proof = Pn + direction_transformed 
            actualizar_grafico(Pf_proof)
            print("Punto de fulcro tras incremento: ", Pf_proof)
            print("Posicion EF tras incremento: ", Pn)
            print("Posicion TCP tras incremento: ", Ptn)
            print(" ")


        #Publicamos cartesian_msg
        self.cartesian_pub.publish(cartesian_msg)

        

