#!/usr/bin/env python
import rospy
import socket
import time
import cv2
import numpy as np
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
    rospy.wait_for_service('ack_service')  # Espera a que se cree el servicio
    try:
        ack_service = rospy.ServiceProxy('ack_service', AckService)  # Crea el objeto referente al servicio
        ip= socket.gethostbyname(socket.gethostname())  # identifica la ip del dispositivo
        groupNumber = 4
        resp = 0
        while resp == 0 or resp.state == 0:
            resp = ack_service(groupNumber, ip)  # Solicita la respuesta del servicio
            time.sleep(1)
        pubEstado.publish(1)
        s = rospy.Service('start_service', StartService,  handle_start_service)
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
        rospy.loginfo("Esperando terminar control")
        while not esperarTerminarRecorrido:
            pass
        pubEstado.publish(4)
        # solicitud_contrasena = rospy.ServiceProxy('iniciar_contrasena', Contrasena) # Crea el objeto referente al servicio
        # data = solicitud_contrasena()
        pubEstado.publish(5)
        endService = rospy.ServiceProxy('end_service', EndService)
        password=handle_contrasena()
        rospy.loginfo(password)
        respFinal = endService(password)

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


def handle_contrasena():
    os.chdir("/home/pi/catkin_ws/src/proyectoFinal/resources")
    os.system("sudo fswebcam -r 1280x720 --no-banner webcam/prueba.jpg")
    ans = ""
    cv2.ml.KNearest_create
    with np.load('knn_data.npz') as data:
        print(data.files)
        train = data['train']
        train_labels = data['train_labels']
    model = cv2.ml.KNearest_create()
    model.train(train, cv2.ml.ROW_SAMPLE, train_labels)
    # Calculo imagen a leer
    im = cv2.imread('./webcam/prueba.jpg')
    # Nueva imagen con contorno
    out = np.zeros(im.shape, np.uint8)
    # Blanco y negro
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    # Thresh nivela la iluminacion de la imagen y reconoce los tonos diferentes
    thresh = cv2.adaptiveThreshold(gray, 250, 1, 1, 11, 2)#(gray,255,1,1,11,2)
    # Contornos en thresh
    _,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    # recorre todos los contornos que encuentre
    # hacer el if en pycharm
    for cnt in contours:
        if cv2.contourArea(cnt)>100:
            [x, y, w, h] = cv2.boundingRect(cnt)
            if h > 120:  # tamano del numero (asi no reconoce cosas pequenas)(obtenido experimentalmente)
                # tamano,color y grosor de rectangulo
                cv2.rectangle(im, (x-20, y-20), (x+w+10, y+h+10), (0, 0, 255), 3)
                roi = thresh[y-20:y+h+10, x-20:x+w+10]
                # Mismo tamano que en el entrenamiento
                roismall = cv2.resize(roi, (20, 20))  # ,interpolation = cv2.INTER_AREA
                roismall = roismall.reshape((1, 400))
                roismall = np.float32(roismall)
                retval, results, neigh_resp, dists = model.findNearest(roismall, k=1)
                string = str(int((results[0][0])))
                print(string)
                ans = ans+string
                # cv2.putText(out,string,(x,y+h),0,1,(0,255,0))
    cv2.imshow("out", im)
    cv2.waitKey(0)
    return int(ans)


if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
