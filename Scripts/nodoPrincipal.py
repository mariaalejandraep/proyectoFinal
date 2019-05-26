#!/usr/bin/env python
import rospy
import socket
import time
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import StartService, EndService
from srv import Contrasena
import TerminarServicio

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


def leviathan():
    global password
    rospy.init_node('nodoPrincipal', anonymous=True)  # inicializa el nodo
    pubEstado = rospy.Publisher('estado', Int32, queue_size=10)
    pubEstado.publish(0)  # Aca publica que esta esperando ack_service 0
    rospy.loginfo("Esperando ack_service")
    rospy.wait_for_service('ack_service')  # Espera a que se cree el servicio
    ack_service = rospy.ServiceProxy('ack_service', Int32)  # Crea el objeto referente al servicio
    # hostname =   # identifica el hostname del dispositivo
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
        # Aca publica que esta esperando start_service 1
        rospy.loginfo ("Enviando start_service")
        rospy.Service('start_service', StartService,  handle_start_service)
        # rospy.spin()
        rospy.wait_for_service('iniciar_recorrido')  # Espera a que se cree el servicio
        iniciar_service = rospy.ServiceProxy('iniciar_recorrido')  # Crea el objeto referente al servicio
        resp.data = 0
        while resp.data == 0:
            resp = iniciar_service(start, goal, n_obstacles, obstacles_array)
        terminar_service = rospy.Service('terminar_recorrido', TerminarServicio, handle_terminar_recorrido)
        while not esperarTerminarRecorrido:
            pass
        # Publica un 4 en topico de estado
        solicitud_contrasena = rospy.ServiceProxy('solicitud_contrasena', Contrasena) # Crea el objeto referente al servicio
        password = solicitud_contrasena()





    except rospy.ServiceException:
        print("Ocurrio un error solicitando servicio")


def handle_start_service(pStart, pGoal, pN_obstacles, pObstacles_array):
    global start, goal, n_obstacles, obstacles_array
    start = pStart
    goal = pGoal
    n_obstacles = pN_obstacles
    obstacles_array = pObstacles_array


def handle_terminar_recorrido():
    global esperarTerminarRecorrido
    esperarTerminarRecorrido = True


if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
