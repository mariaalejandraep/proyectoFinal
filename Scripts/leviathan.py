#!/usr/bin/env python
#Las librerias que se importan
import rospy
import RPi.GPIO as GPIO

#Es la tasa en Hertz del nodo.
h = 10

def leviathan():

    rate = rospy.Rate(h)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        leviathan()
    except rospy.ROSInterruptException:
        pass
