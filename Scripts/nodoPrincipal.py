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
    #pubEstado = rospy.Publisher('estado', Int32, queue_size=10)
    #pubEstado.publish(0)  # Aca publica que esta esperando ack_service 0
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
        #pubEstado.publish(1)

        s = rospy.Service('start_service', StartService,  handle_start_service)

        rospy.loginfo("Despues de start service")

        rospy.loginfo("Enviando iniciar_recorrido")
        rospy.wait_for_service('iniciar_recorrido')

        while not esperarStartService:
            pass

        iniciar_service = rospy.ServiceProxy('iniciar_recorrido', StartService)  # Crea el objeto referente al servicio
        iniciar_service(escenario)

        rospy.Service('terminar_recorrido', TerminarRecorrido, handle_terminar_recorrido)

        rospy.loginfo("Esperando terminar recorrido")


        while not esperarTerminarRecorrido:
            pass

        #pubEstado.publish(4)
        solicitud_contrasena = rospy.ServiceProxy('solicitud_contrasena', Contrasena) # Crea el objeto referente al servicio
        password = solicitud_contrasena()
        #pubEstado.publish(5)
        resp.data = password
        end_service = rospy.ServiceProxy('end_service', Int32)
        respFinal = end_service(resp)
        if respFinal.data == 1:
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
    return None


def handle_terminar_recorrido():
    global esperarTerminarRecorrido
    esperarTerminarRecorrido = True


if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
