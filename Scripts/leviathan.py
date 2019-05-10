#!/usr/bin/env python
#Las librerias que se importan
import rospy
import RPi.GPIO as GPIO

#Es la tasa en Hertz (Hz) del nodo.
h = 10

#Variable con el pin que va al motor.
pwm1 = 10

#Frecuencia en Hertz (Hz) del pin que va al motor.
f1 = 500

#Ciclo util del pulso. Un numero entre 0 y 100.
ciclo = 0

def prender():
    rospy.loginfo("El ciclo util es: {}".format(ciclo))
    p.changeDutyCycle(ciclo)

def leviathan():
    rospy.init_node('Raspberry_controller', anonymous=True)

    rate = rospy.Rate(h)
    p = GPIO.PWM(pwm1,f1)
    p.start(0)

    while not rospy.is_shutdown():
        prender()
        rate.sleep()

if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
