#!/usr/bin/python2
import cv2
import numpy as np
import os
os.chdir('../resources')
#os.system("sudo fswebcam -r 1280x720 --no-banner webcam/prueba.jpg")

################################Entrenamiento
#entrenamiento con dato del archivo intentoEntrenamiento.py
cv2.ml.KNearest_create

with np.load('knn_data.npz') as data:
    print( data.files )
    train = data['train']
 
    
    train_labels = data['train_labels']
   
	
global knn


model = cv2.ml.KNearest_create()
model.train(train,cv2.ml.ROW_SAMPLE,train_labels)

#################################Calculo
#imagen a leer
im = cv2.imread('./webcam/nuevoNums.jpg')

#nueva imagen con contorno
out = np.zeros(im.shape,np.uint8)
#blanco y negro
gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
#thresh nivela la iluminacion de la imagen y reconoce los tonos diferentes
thresh = cv2.adaptiveThreshold(gray,250,1,1,11,2)#(gray,255,1,1,11,2)


#contornos en thresh
_,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
# recorre todos los contornos que encuentre

#hacer el if en pycharm
for cnt in contours:
    if cv2.contourArea(cnt)>100:
        [x,y,w,h] = cv2.boundingRect(cnt)
        if  h>150:#tamano del numero (asi no reconoce cosas pequenas)(obtenido experimentalmente)

#tamano,color y grosor de rectangulo
            cv2.rectangle(im,(x-20,y-20),(x+w+10,y+h+10),(0,0,255),3)

            roi = thresh[y-20:y+h+10,x-20:x+w+10]
	    
#mismo tamano que en el entrenamiento
	    
            roismall = cv2.resize(roi,(20,20))#,interpolation = cv2.INTER_AREA

	    

	    roismall = roismall.reshape((1,400))


            roismall = np.float32(roismall)
            retval, results, neigh_resp, dists = model.findNearest(roismall, k = 1) 
            string = str(int((results[0][0])))
            print (string)
            #cv2.putText(out,string,(x,y+h),0,1,(0,255,0))


cv2.imshow('out',out)
cv2.waitKey(0)

def contrasena():
    rospy.init_node ('nodo_Contrasena', anonymous=True)
    rate = rospy.Rate (10)
    s = rospy.Service ('iniciar_Contrasena', StartService, handle_contrasena)

    rospy.loginfo ("Despues de")

    # Se espera a que se publique por primera vez a traves del topico preguntarCasillas
    while not empezar:
        pass

    inicializar ()

    while not rospy.is_shutdown ():
        rate.sleep ()

def handle_iniciar_contrsena():
    def inicializar():
        global pubPos, pubCov, t
        pubPos = rospy.Publisher ('robot_position', Pose, queue_size=10)
        pubCov = rospy.Publisher ('robot_uncertainty', Covariance, queue_size=10)
        rospy.Subscriber ('velocidad_deseada', Float32MultiArray, actualizar)
        t = [time.time (), time.time ()]
if __name__ == '__main__':
    try:
        contrasena()
    except rospy.ROSInterruptException:
        pass
