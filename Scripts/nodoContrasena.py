#!/usr/bin/python2
import cv2
import numpy as np
import os
import rospy
from proyectoFinal.srv import Contrasena


def handle_contrasena(req):
    global knn

    os.chdir("/home/pi/catkin_ws/src/proyectoFinal/resources")
    os.system("sudo fswebcam -r 1280x720 --no-banner webcam/prueba.jpg")
    os.system("raspberry")
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


def contrasena():
    rospy.init_node ('nodo_Contrasena', anonymous=True)
    rate = rospy.Rate (10)
    # handle_contrasena(0)
    s = rospy.Service ('iniciar_contrasena', Contrasena, handle_contrasena)
    rospy.loginfo ("Despues de")
    rospy.spin()
    #while not rospy.is_shutdown ():
    #    rate.sleep ()


if __name__ == '__main__':
    try:
        contrasena()
    except rospy.ROSInterruptException:
        pass
