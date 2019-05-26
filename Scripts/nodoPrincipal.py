#!/usr/bin/env python
import rospy
import socket
import time
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import StartService, EndService
from proyectoFinal.srv import Contrasena
from proyectoFinal.srv import TerminarRecorrido

# Posicion de inicio del robot
start = Pose()
# Posicion final a la que se quiere llegar
goal = Pose()
# Numero de obstaculos que se envian
n_obstacles = Int32()
# Objeto tipo Obstacle con informacion de los obstaculos
obstacles_array = Obstacle()
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
    # rospy.wait_for_service('ack_service')  # Espera a que se cree el servicio
    rospy.loginfo ("Esperando ack_service")
    ack_service = rospy.ServiceProxy('ack_service', Int32)  # Crea el objeto referente al servicio
    ip = String()
    ip.data = socket.gethostbyname(socket.gethostname())  # identifica la ip del dispositivo
    groupNumber = Int32()
    groupNumber.data = 4
    resp = Int32()
    resp.data = 0
    try:
        while resp.data == 0:
            resp = ack_service(groupNumber, ip)  # Solicita la repsuesta del servicio
            if resp.data != 1:
                resp.data = 0
            time.sleep(1)
        rospy.loginfo("Enviando start_service")
        pubEstado.publish(1)
        rospy.Service('start_service', StartService,  handle_start_service)
        rospy.wait_for_service('iniciar_recorrido')  # Espera a que se cree el servicio
        while not esperarStartService:
            pass
        rospy.wait_for_service('iniciar_recorrido')
        iniciar_service = rospy.ServiceProxy('iniciar_recorrido')  # Crea el objeto referente al servicio
        iniciar_service(start, goal, n_obstacles, obstacles_array)
        rospy.Service('terminar_recorrido', TerminarRecorrido, handle_terminar_recorrido)
        while not esperarTerminarRecorrido:
            pass
        pubEstado.publish(4)
        solicitud_contrasena = rospy.ServiceProxy('solicitud_contrasena', Contrasena) # Crea el objeto referente al servicio
        password = solicitud_contrasena()
        pubEstado.publish(5)
        resp.data = password
        end_service = rospy.ServiceProxy('end_service', Int32)
        respFinal = end_service(resp)
        if respFinal.data == 1:
            print("Drop the mic, leave the room")
        else:
            print("Burn the room")
    except rospy.ServiceException:
        print("Ocurrio un error solicitando servicio")


def handle_start_service(pStart, pGoal, pN_obstacles, pObstacles_array):
    global start, goal, n_obstacles, obstacles_array, esperarStartService
    start = pStart
    goal = pGoal
    n_obstacles = pN_obstacles
    obstacles_array = pObstacles_array
    esperarStartService = True


def handle_terminar_recorrido():
    global esperarTerminarRecorrido
    esperarTerminarRecorrido = True


if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
