#!/usr/bin/env python
import rospy
import socket
import time
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import StartService, EndService
import TerminarServicio


iniciarMov = False
start = Pose()
goal = Pose()
n_obstacles = Int32()
obstacles_array = Obstacle()
password = 0


def leviathan():
    rospy.wait_for_service('ack_service')  # Espera a que se cree el servicio
    rospy.init_node('nodoPrincipal', anonymous=True)  # inicializa el nodo
    ack_service = rospy.ServiceProxy('ack_service', Int32)  # Crea el objeto referente al servicio
    hostname = socket.gethostname()  # identifica el hostname del dispositivo
    ip = String()
    ip.data = socket.gethostbyname(hostname)  # identifica la ip del dispositivo
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
        start_service = rospy.Service('start_service', StartService,  handle_start_service)
        # rospy.spin()
        rospy.wait_for_service('iniciar_recorrido')  # Espera a que se cree el servicio
        iniciar_service = rospy.ServiceProxy('iniciar_recorrido', Int32)  # Crea el objeto referente al servicio
        resp.data = 0
        while resp.data == 0:
            resp = iniciar_service(start, goal, n_obstacles, obstacles_array)
        terminar_service = rospy.Service('terminar_recorrido', TerminarServicio, handle_terminar_recorrido)

    except rospy.ServiceException:
        print("Ocurrio un error solicitando servicio")


def handle_start_service(pStart, pGoal, pN_obstacles, pObstacles_array):
    global iniciarMov, start, goal, n_obstacles, obstacles_array
    iniciarMov = True
    start = pStart
    goal = pGoal
    n_obstacles = pN_obstacles
    obstacles_array = pObstacles_array


def handle_terminar_recorrido(pPassword):
    global password
    password = pPassword


if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
