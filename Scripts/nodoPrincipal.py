#!/usr/bin/env python
import rospy
import socket
import time
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose
#from master_msgs_iele3338.msg import Obstacle
#from master_msgs_iele3338.srv import StartService, EndService
from master_msgs_iele3338.srv import *
from master_msgs_iele3338.msg import *
from proyectoFinal.srv import Contrasena
from proyectoFinal.srv import TerminarRecorrido
import os

#Variable con toda la informacion del escenario
escenario = None
# Posicion de inicio del robot
start = Pose()
# Posicion final a la que se quiere llegar
goal = Pose()
# Numero de obstaculos que se envian
n_obstacles = 0
# Objeto tipo Obstacle con informacion de los obstaculos
obstacles_array = []
# Contrasena obtenida
password = None
# Variable que identifica si se termino el recorrido
esperarTerminarRecorrido = False
# Variable que identifica que se solicito el servicio start_service
esperarStartService = False


def leviathan():
    global password
    rospy.init_node('nodoPrincipal', anonymous=True)  # inicializa el nodo
    pubEstado = rospy.Publisher('estado', Int32, queue_size=10)
    pubEstado.publish(0)  # Aca publica que esta esperando ack_service 0
    rospy.loginfo("Llega.")
    rospy.wait_for_service('ack_service')  # Espera a que se cree el servicio
    rospy.loginfo("Esperando ack_service")
    try:
        ack_service = rospy.ServiceProxy('ack_service', AckService)  # Crea el objeto referente al servicio
        ip= socket.gethostbyname(socket.gethostname())  # identifica la ip del dispositivo
        groupNumber = 4
        resp = 0
        while resp == 0 or resp.state == 0:
            resp = ack_service(groupNumber, ip)  # Solicita la respuesta del servicio
            print(resp.state)
            time.sleep(1)
        rospy.loginfo("Enviando start_service")
        pubEstado.publish(1)

        s = rospy.Service('start_service', StartService,  handle_start_service)

        rospy.loginfo("Despues de start service")

        rospy.loginfo("Enviando iniciar_recorrido e iniciar_odometria")
        rospy.wait_for_service('iniciar_recorrido')
        rospy.wait_for_service('iniciar_odometria')
        rospy.wait_for_service('iniciar_encoders')

        while not esperarStartService:
            pass

        iniciar_service = rospy.ServiceProxy('iniciar_recorrido', StartService)  # Crea el objeto referente al servicio
        iniciar_service(escenario)

        iniciar_odometria = rospy.ServiceProxy('iniciar_odometria', StartService)
        iniciar_odometria(escenario)

        iniciar_encoders = rospy.ServiceProxy('iniciar_encoders', StartService)
        iniciar_encoders(escenario)

        # rospy.Subscriber ('termino_recorrido', Int32, handle_terminar_recorrido)

        s1=rospy.Service('terminar_control', TerminarRecorrido, handle_terminar_recorrido)
        rospy.spin()
        rospy.loginfo("Esperando terminar control")

        while not esperarTerminarRecorrido:
            rospy.loginfo ("Esta esperando actualmente que termine")
            pass

        rospy.loginfo ("Recibio el final el recorrido y paso el while")

        pubEstado.publish(4)
        solicitud_contrasena = rospy.ServiceProxy('iniciar_contrasena', Contrasena) # Crea el objeto referente al servicio
        data = solicitud_contrasena()
        pubEstado.publish(5)

        rospy.loginfo(password)

        endService = rospy.ServiceProxy('end_service', EndService)
        respFinal = endService(data.password)

        if respFinal == 1:
            print("Drop the mic, leave the room")
        else:
            print("Burn the room")
    except rospy.ServiceException:
        print("Ocurrio un error solicitando servicio")


def handle_start_service(startS):
    global escenario, start, goal, n_obstacles, obstacles_array, esperarStartService

    escenario = startS
    start = startS.start
    goal = startS.goal
    n_obstacles = startS.n_obstacles
    obstacles_array = startS.obstacles
    esperarStartService = True
    rospy.loginfo("Handle")
    return []


def handle_terminar_recorrido(req):
    global esperarTerminarRecorrido
    esperarTerminarRecorrido = True
    rospy.loginfo ("Recibio el llamado del topico para terminar recorrido")
    return []


if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
