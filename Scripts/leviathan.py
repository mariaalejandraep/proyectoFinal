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

#Ciclo útil del pulso. Un número entre 0 y 100.
ciclo = 1

def prender():
    p.changeDutyCycle(ciclo)

def leviathan():

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
