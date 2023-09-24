#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import csv
from conversions import matrix_from_quaternion
from conversions import quaternion_from_matrix 
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition

class iiwa_surgery:
    """!
    Clase para controlar un robot quirúrgico Kuka en un entorno de ROS.    
    Esta clase permite configurar el robot, establecer datos de la herramienta y mover el robot tanto en espacio de articulación como en espacio cartesiano.
    """
    
    def __init__(self, simulation_mode=True):
        """!
        Inicializa una instancia de la clase iiwa_surgery.
        @param simulation_mode (bool, opcional): Indica si se está ejecutando en modo simulación. El valor predeterminado es True.
        @protected
        """
        
        # Variable miembro para configurar el modo de trabajo (simulación o robot real)
        ## @protected
        # Booleano que indica si el sistema está en modo de simulación.
        self.simulation_mode = simulation_mode

        # Variables miembro para los datos de la herramienta
        ## @protected
        # Valor que representa la longitud de la herramienta.
        self.tool_length = 0.0

        ## @protected
        # Lista que almacena la orientación de la herramienta en formato [roll, pitch, yaw].
        self.tool_orientation = [0.0, 0.0, 0.0]

        # Variable miembro para el punto de fulcro
        ## @protected
        # Valor que representa la posición del punto de fulcro, un número entre 0 y 1.
        self.fulcrum_fi = 0.0

        # Variables miembro con el valor de la IP del robot
        ## @protected
        # Cadena que representa la dirección IP del robot.
        self.robot_ip = ""

        # Variables miembro con el modo de trabajo del robot
        ## @protected
        # Cadena que indica el modo de trabajo del robot ('free' o 'pivot').
        self.work_mode = "free"  # Valor predeterminado
        
        # Otras variables miembro de configuración del robot, si es necesario

        ## @protected
        # Publicador ROS que se utiliza para enviar mensajes de posición cartesiana al robot.
        self.cartesian_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)

        ## @protected
        # Publicador ROS que se utiliza para enviar mensajes de posición articular al robot.
        self.joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=10)
        
        # ... otros publicadores y suscriptores
  
    def set_tool_data(self, tool_length, tool_orientation):
        """!
        Configura los datos de la herramienta.
        @param tool_length (float): Longitud de la herramienta.
        @param tool_orientation (list): Orientación de la herramienta en formato [roll, pitch, yaw].
        @protected
        """
        
        # Método para configurar los datos de la herramienta
        # Variables miembro para los datos de la herramienta
        self.tool_length = tool_length
        self.tool_orientation = tool_orientation
        
    def set_fulcrum_fi(self, fulcrum_fi):
        """!
        Configura el punto de fulcro.
        @param fulcrum_fi (float): Posición del punto de fulcro, un valor entre 0 y 1.
        @protected
        """
        
        # Método para configurar el punto de fulcro
        if 0 <= fulcrum_fi <= 1:
            # Variable miembro para el punto de fulcro
            self.fulcrum_fi = fulcrum_fi
        else:
            rospy.logwarn("El punto de fulcro debe ser un valor decimal entre 0 y 1.")
        
    def set_robot_ip(self, robot_ip):
        """!
        Configura la dirección IP del robot.
        @param robot_ip (str): Dirección IP del robot.
        @protected
        """
        
        # Variables miembro con el valor de la IP del robot
        self.robot_ip = robot_ip

    def set_work_mode(self, work_mode):
        """!
        Configura el modo de trabajo del robot.
        @param work_mode (str): Modo de trabajo del robot ('free' o 'pivot').
        @protected
        """
        
        if work_mode in ["free", "pivot"]:
            self.work_mode = work_mode
        else:
            rospy.logwarn("Modo de trabajo no válido.")

        
    def move_joint(self, joint_config):
        """!
        Mueve el robot en el espacio de articulación.
        @param joint_config (iiwa_msgs.msg.JointPosition): Configuración de las articulaciones del robot.
        @protected
        """
        
        joint_msg = JointPosition()
        joint_msg = joint_config
        self.joint_pub.publish(joint_msg)
 
    def move_cartesian(self, pose):
        """!
        Mueve el robot en el espacio cartesiano.
        @param pose (geometry_msgs.msg.PoseStamped): Posición y orientación deseada del Tool Center Point (TCP) en espacio cartesiano.
        @protected
        """
        
        cartesian_msg = PoseStamped()

        # Obtener el EF a partir del TCP:

        # Obtener la posición y orientación del mensaje
        position = pose.pose.position
        orientation = pose.pose.orientation

        # Reordenar el cuaternio en el vector q (w, x, y, z)
        q = [orientation.w, orientation.x, orientation.y, orientation.z]
         
        # Crear una matriz de rotación a partir del quaternio recibido
        Rm = matrix_from_quaternion(q)

        # Calcular el vector de dirección relativo al TCP
        direction = np.array([0,0,-self.tool_length])
        direction_transformed = np.dot(Rm, direction)

        # Guardar la posición del TCP 
        Ptcp = np.array([position.x,position.y,position.z])
        
        # Calcular la posición del EF 
        Pef = Ptcp + direction_transformed

        # Actualizar los valores de la posición del EF
        position.x = Pef[0]
        position.y = Pef[1]
        position.z = Pef[2]

        # Actualizar el mensaje de cartesian_msg con los valores ajustados
        cartesian_msg.header = pose.header
        cartesian_msg.pose.position = position
        cartesian_msg.pose.orientation = orientation #EF tendrá la misma orientación que el TCP
        
        # Publicar el mensaje 
        self.cartesian_pub.publish(cartesian_msg)

    def move_cartesian_fulcrum(self, pose, increment_vector, j):
        """!
        Mueve el robot en el espacio cartesiano alrededor de un punto de fulcro.
        @param pose (geometry_msgs.msg.PoseStamped): Posición y orientación deseada del Tool Center Point (TCP) en espacio cartesiano.
        @param increment_vector (list): Vector de incrementos [Ph1, Ph2, Ph3] para mover la herramienta.
        @param j (int): Indicador de posición, 0 para la posición inicial, 1 para las posiciones sucesivas.
        @protected
        """
        
        cartesian_msg = PoseStamped() # En este mensaje se almacenará la información del EF 

        nombre_archivo = "/home/javilara/iiwa_stack_ws/src/iiwa_stack/iiwa_surgery/src/datos_robot.csv"

        def guardar_datos_csv(datos, nombre_archivo):
            """
            Función para guardar datos en un archivo CSV.
            @param datos (list): Lista de datos a ser guardados en el archivo.
            @param nombre_archivo (str): Ruta al archivo CSV.
            @protected
            """
            
            with open(nombre_archivo, mode='a') as archivo_csv:
                escritor_csv = csv.writer(archivo_csv)
                escritor_csv.writerow(datos)
      
        # Asignar los valores de increment_vector a Ph1, Ph2 y Ph3. Éstos serán los incrementos para mover la herramienta
        Ph1 = increment_vector[0]
        Ph2 = increment_vector[1]
        Ph3 = increment_vector[2]      

        if(j == 0): #Si es la primera posición que se recibe:

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

            # Guardamos la posicion del TCP inicial
            ## @protected
            # Vector que almacena la posición del TCP (Tool Center Point) en el espacio cartesiano.
            self.Ptcp = np.array([position.x,position.y,position.z])

            # Calcular la posición del EF inicial
            Pef = self.Ptcp + direction_transformed
  
            # Calcular el vector de dirección relativo al EF 
            direction = np.array([0,0, self.fulcrum_fi*self.tool_length])
            direction_transformed = np.dot(Rm, direction)
        
            # Calcular la posición del punto de fulcro
            ## @protected
            # Vector que almacena la posición del punto de fulcro en el espacio cartesiano.
            self.Pf = Pef + direction_transformed # El punto de fulcro se calcula al principio y no cambia

            # Se muestran por pantalla los datos 
            print("Punto de fulcro: ", self.Pf)
            print("Posicion EF inical: ", Pef)
            print("Posicion TCP inicial: ", self.Ptcp)
            print(" ")

            # Se guardan los datos en el archivo .csv
            guardar_datos_csv([self.Pf[0], self.Pf[1], self.Pf[2]], nombre_archivo)
            guardar_datos_csv([Pef[0], Pef[1], Pef[2]], nombre_archivo)
            guardar_datos_csv([self.Ptcp[0], self.Ptcp[1], self.Ptcp[2]], nombre_archivo)
            

            # Actualizar la información del cartesian_msg (EF)
            cartesian_msg.header = pose.header
            cartesian_msg.pose.position.x = Pef[0]
            cartesian_msg.pose.position.y = Pef[1]
            cartesian_msg.pose.position.z = Pef[2]
            cartesian_msg.pose.orientation.w = q[0]
            cartesian_msg.pose.orientation.x = q[1]
            cartesian_msg.pose.orientation.y = q[2]
            cartesian_msg.pose.orientation.z = q[3]

        if(j == 1): # Si no es la primera posición que se recibe:
        
            # Nueva posición de la punta de la herramienta con el incremento
            Ptn = [self.Ptcp[0]+Ph1, self.Ptcp[1]+Ph2, self.Ptcp[2]+Ph3]

            # Nueva direccion en el eje z de la herramienta
            zn = self.Pf - Ptn

            # Distancia del punto de fulcro a la nueva posición de la pinza
            Mzn = math.sqrt(zn[0]*zn[0] + zn[1]*zn[1] + zn[2]*zn[2])
            ro = self.tool_length - Mzn

            # Nueva posición del efector final del robot
            Pn = self.Pf + ro*zn/Mzn

            # El eje z de la herramienta ya lo tenemos, pero se hace unitario.
            znn = [-zn[0]/Mzn, -zn[1]/Mzn, -zn[2]/Mzn]

            # Calcular un vector perpendicular al eje Z estándar y a znn
            xnn = np.cross([0, 0, 1], znn)
            xnn /= np.linalg.norm(xnn)  # Normaliza el vector

            # Calcula un segundo vector perpendicular a znn y xnn
            ynn = np.cross(znn, xnn)
            ynn /= np.linalg.norm(ynn)  # Normaliza el vector

            # Crea una matriz de rotación a partir de los vectores unitarios
            M = np.column_stack((xnn, ynn, znn))            

            # Calcular la orientación en cuaternios a partir de matriz de rotación
            q = quaternion_from_matrix(M)   
                    
            # Actualizar la información del cartesian_msg (nuevo EF)
            cartesian_msg.header = pose.header
            cartesian_msg.pose.position.x = Pn[0]
            cartesian_msg.pose.position.y = Pn[1]
            cartesian_msg.pose.position.z = Pn[2]
            cartesian_msg.pose.orientation.w = q[0]
            cartesian_msg.pose.orientation.x = q[1]
            cartesian_msg.pose.orientation.y = q[2]
            cartesian_msg.pose.orientation.z = q[3]

            # Se muestran por pantalla los datos             
            print("Posicion EF tras incremento: ", Pn)
            print("Posicion TCP tras incremento: ", Ptn)
        
            # Imprime la distancia en valor absoluto
            distancia = np.linalg.norm(Pn - Ptn)
            print("Longitud de la herramienta: ",abs(distancia))

            # Se guardan los datos en el archivo .csv
            guardar_datos_csv([Pn[0], Pn[1], Pn[2]], nombre_archivo)
            guardar_datos_csv([Ptn[0], Ptn[1], Ptn[2]], nombre_archivo)

            #COMPROBACIONES
            
            # Para comprobar que se han hecho bien los calculos y el movimiento alrededor del punto de fulcro se comprueba 
            # si el punto de fulcro (self.Pf) se encuentra en la herramienta en la nueva posición, es decir, 
            # entre Pn y Ptn o lo que es lo mismo, entre las nuevas posiciones de EF y TCP

            epsilon = 1e-6  # Valor pequeño para manejar errores de punto flotante

            # Calcula las distancias entre los puntos
            dist1 = np.linalg.norm(np.array(Pn) - np.array(self.Pf))
            dist2 = np.linalg.norm(np.array(Ptn) - np.array(self.Pf))
            total_dist = np.linalg.norm(np.array(Ptn) - np.array(Pn))

            # Comprueba si el punto fijado como punto de fulcro se encuentra entre Pn y Ptn, teniendo en cuenta una pequeña tolerancia
            is_between=abs(dist1 + dist2 - total_dist) < epsilon

            if is_between:
                print("El punto de fulcro (Pf) está entre EF (Pn) y TCP (Ptn).")
            else:
                print("El punto de fulcro (Pf) no está entre EF (Pn) y TCP (Ptn)")
            print(" ")


        # Publicar el mensaje 
        self.cartesian_pub.publish(cartesian_msg)
        

        
