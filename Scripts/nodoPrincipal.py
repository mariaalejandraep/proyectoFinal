#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from master_msgs_iele3338.msg import Obstacle

iniciarMov = False
start = Pose()
goal = Pose()
n_obstacles = Int32()
obstacles_array = Obstacle()


def leviathan():
    rospy.wait_for_service('ack_service')  # Espera a que se cree el servicio
    rospy.init_node('raspberry_nodo_principal', anonymous=True)  # inicializa el nodo
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
        start_service = rospy.Service('start_service', handle_start_service)
        rospy.spin()

    except rospy.ServiceException:
        print("Ocurrio un error solicitando servicio")


def handle_start_service(pStart, pGoal, pN_obstacles, pObstacles_array):
    global iniciarMov, start, goal, n_obstacles, obstacles_array
    iniciarMov = True
    start = pStart
    goal = pGoal
    n_obstacles = pN_obstacles
    obstacles_array = pObstacles_array


if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
