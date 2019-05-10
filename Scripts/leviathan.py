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


#Frecuencia en Hertz (Hz) del pin que va al motor.
f = 500

#Ciclo util del pulso para el motor A. Un numero entre 0 y 100.
cicloA = 10

#Ciclo util del pulso para el motor B. Un numero entre 0 y 100.
cicloB = 20

def prender():
    global p1, p2
    rospy.loginfo("El ciclo util es: {}".format(ciclo))
    GPIO.output(pwmA1,1)
    GPIO.output(pwmB2,1)
    p1.ChangeDutyCycle(cicloA)
    p2.ChangeDutyCycle(cicloB)

def leviathan():
    global p1, p2
    rospy.init_node('Raspberry_controller', anonymous=True)
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(pwmA1,GPIO.OUT)
    GPIO.setup(pwmA2,GPIO.OUT)
    GPIO.setup(pwmB1,GPIO.OUT)
    GPIO.setup(pwmB2,GPIO.OUT)

    rate = rospy.Rate(h)
    GPIO.output(pwmA1,0)
    GPIO.output(pwmB2,0)

    p1 = GPIO.PWM(pwmA2,f)
    p2 = GPIO.PWM(pwmB1,f)
    p1.start(0)
    p2.start(0)
    # try:
         # service = rospy.ServicePr
    while not rospy.is_shutdown():
        prender()
        rate.sleep()

if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
