#!/usr/bin/env python
#Las librerias que se importan
import rospy
import RPi.GPIO as GPIO

#Es la tasa en Hertz (Hz) del nodo.
h = 10

#Variable con el primer pin que va al driver para controlar el motor A.
pwmA1 = 11

#Variable con el segundo pin que va al driver para controlar el motor A.
pwmA2 = 12

#Variable con el primer pin que va al driver para controlar el motor B.
pwmB1 = 15

#Variable con el segundo pin que va al driver para controlar el motor B.
pwmB2 = 16

# Variable para pin de lectura de encoder A 1

encodA1= 35

#Variable para pin de lectura de encoder A 2

encodA2= 36

# Variable para pin de lectura de encoder B 1

encodB1= 37

#Variable para pin de lectura de encoder B 2

encodB2= 38

#Frecuencia en Hertz (Hz) del pin que va al motor.
f = 500

#Ciclo util del pulso para el motor A. Un numero entre 0 y 100.
cicloA = 5

#Ciclo util del pulso para el motor B. Un numero entre 0 y 100.
cicloB = 5

contadorA = 0
contadorB = 0

def prender():
    #global #p1, p2#, contadorA, contadorB, eA1, eA2, eB1, eB2, contadorA, contadorB
    #rospy.loginfo("El ciclo util A es: {}".format(cicloA))
    #rospy.loginfo("El ciclo util B es: {}".format(cicloB))

    GPIO.output(pwmA1,1)
    GPIO.output(pwmB2,1)
    #p1.ChangeDutyCycle(cicloA)
    #p2.ChangeDutyCycle(cicloB)

    # eA1 = GPIO.input(encodA1)
    # eA2 = GPIO.input(encodA2)
    # eB1 = GPIO.input(encodB1)
    # eB2 = GPIO.input(encodB2)

    #
    # if eA1 or eA2:
    #     contadorA=contadorA+1
    # if eB1 or eB2:
    #     contadorB=contadorB+1
    # if contadorA==6 or contadorB==6:
    #     contadorA = 0
    #     contadorB = 0
    #     rospy.loginfo("es true perras")


def apagar():
    #cicloA = 0
    #cicloB = 0
    #p1.ChangeDutyCycle(cicloA)
    #p2.ChangeDutyCycle(cicloB)
    rospy.loginfo("Apagando. El ciclo util de A es: {}".format(cicloA))
    rospy.loginfo("Apagando. El ciclo util de B es: {}".format(cicloB))
    GPIO.cleanup()

def leviathan():
    #global #p1, p2#, eA1, eA2, eB1, eB2
    rospy.init_node('Raspberry_controller', anonymous=True)
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(pwmA1,GPIO.OUT)
    GPIO.setup(pwmA2,GPIO.OUT)
    GPIO.setup(pwmB1,GPIO.OUT)
    GPIO.setup(pwmB2,GPIO.OUT)

    # GPIO.setup(encodA1,GPIO.IN)
    # GPIO.setup(encodA2,GPIO.IN)
    # GPIO.setup(encodB1,GPIO.IN)
    # GPIO.setup(encodB2,GPIO.IN)

    rate = rospy.Rate(h)

    GPIO.output(pwmA1,0)
    GPIO.output(pwmA2,0)
    GPIO.output(pwmB1,0)
    GPIO.output(pwmB2,0)

    #p1 = GPIO.PWM(pwmA2,f)
    #p2 = GPIO.PWM(pwmB1,f)

    #p1.start(0)
    #p2.start(0)
    # try:
         # service = rospy.ServicePr
    while not rospy.is_shutdown():
        prender()
        rate.sleep()

    apagar()

if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
