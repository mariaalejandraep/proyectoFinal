#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, roslaunch, time
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from master_msgs_iele3338.msg import Covariance
import numpy as np

pos = Pose()
cov = Covariance()

#El primero es la velocidad de la rueda izquierda y el segundo es la velocidad de la derecha.
vel = [0, 0]

dt = 0
dSl = 0
dSr = 0
dS = 0
dScos=0
dSsin=0
cose=0
seno=0

#Averiguar que es b. Por ahora en 1.
b = 1

#Constantes de la matriz de covarianza. Por ahora en 1.
kr = 1
kl = 1

#Theta. Es la orientacion del robot con respecto al eje z.
O = 0
dO = 0

#Es la tasa en Hz del nodo.
h = 10


#Covarianza
Covarianza=np.matrix([[0,0,0],[0,0,0],[0,0,0]])
CovarSrSl=np.matrix([[0,0],[0,0]])
Fpt1=np.matrix([[0, 0, 0],[0, 0, 0],[0, 0, 0]])
Fpt1trans=np.transpose(Fpt1)
FdS=np.matrix([[0,0],[0,0],[0,0]])
FdStrans=np.transpose(Fpt1)

t = [0, 0]

def odometria():
    #rospy.wait_for_service()
    rospy.init_node('nodo_odometria', anonymous = True)

    inicializar()


def inicializar():
    global pubPos, pubCov, t
    pubPos = rospy.Publisher('robot_position', Pose, queue_size = 10)
    pubCov = rospy.Publisher('robot_uncertainty', Covariance, queue_size = 10)
    rospy.Suscriber('velocidad_deseada', Float32MultiArray, actualizar)
    t = [time.time(), time.time()]

def actualizar(msg):
    global pos,cov, vel, dt, dS, dSl, dSr, O, dO, Covarianza,pubPos,pubCov,CovarSrSl,CovarSrSl,Fpt1,Fpt1trans,FdS,FdStrans

    t.append(time.time())
    t = t[-2:]

    dt = (t[1]-t[0])

    dSl = vel[0]*dt
    dSr = vel[1]*dt

    dS = (dSl+dSr)/2
    dO = (dSr-dSl)/b

    O = pos.orientation.z

    cose=math.cos(O+dO/2.0)
    seno=math.sin(O+dO/2.0)
    dScos=dS*cose
    dSsin= dS*seno

    pos.position.x = pos.position.x + dScos
    pos.position.y = pos.position.y +dSsin
    pos.orientation.z = O + dO

    vel = msg.data


    CovarSrSl=np.matrix([kr*np.absolute(dSr),0],[0,kl*np.absolute(dSl)],dtype='f')

    Fpt1=np.matrix([[1, 0, -dSsin],[1, 0, dScos],[0, 0, 1]])
    Fpt1trans=np.transpose(Fpt1)

    FdS=np.matrix([[(1/2)*cose-(1/(2*b))*dSsin,(1/2)*cose+(1/(2*b))*dSsin],[(1/2)*seno+(1/(2*b))*dScos,(1/2)*seno-(1/(2*b))*dScos],[1/b,-(1/b)]])
    FdStrans=np.transpose(Fpt1)

    Covarianza=np.sum(np.multiply(np.multiply(Fpt1,Covarianza),Fpt1trans),np.multiply(np.multiply(FdS,CovarSrSl),FdStrans))

    cov.sigma11=Covarianza[0,0]
    cov.sigma12=Covarianza[0,1]
    cov.sigma13=Covarianza[0,2]
    cov.sigma21=Covarianza[1,0]
    cov.sigma22=Covarianza[1,1]
    cov.sigma23=Covarianza[1,2]
    cov.sigma31=Covarianza[2,0]
    cov.sigma32=Covarianza[2,1]
    cov.sigma33=Covarianza[2,2]

    pubPos.publish(pos)
    pubCov.publish(cov)






if __name__ == '__main__':
    try:
        # En caso de que se pasen tres parametros tipo numero se ajusta la nueva posicion final deseada
        odometria()
    except rospy.ROSInterruptException:
        pass
