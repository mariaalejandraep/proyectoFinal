#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32
# LED A ES EL DE ARRIBA Y LED B ES EL DE ABAJO
# Variable que controla color 1 del LED A
LedA_1 = 29
# Variable que controla color 2 del LED A
LedA_2 = 24
# Variable que controla color 3 del LED A
LedA_3 = 26
# Variable que controla color 1 del LED B
LedB_1 = 32
# Variable que controla color 2 del LED B
LedB_2 = 33
# Variable que controla color 3 del LED B
LedB_3 = 31
# Frecuencia Leds
fLeds = 500


def setPins():
    GPIO.setmode(GPIO.BOARD)
    # Configurando los pines de salida para LEDs
    GPIO.setup(LedA_1, GPIO.OUT)
    GPIO.setup(LedA_2, GPIO.OUT)
    GPIO.setup(LedA_3, GPIO.OUT)
    GPIO.setup(LedB_1, GPIO.OUT)
    GPIO.setup(LedB_2, GPIO.OUT)
    GPIO.setup(LedB_3, GPIO.OUT)
    # Configurando senales de salida para los LEDS e inicializandolas en ciclo util de 0
    GPIO.output (LedA_1, GPIO.LOW)
    GPIO.output (LedA_2, GPIO.HIGH)
    GPIO.output (LedA_3, GPIO.HIGH)
    GPIO.output (LedB_1, GPIO.LOW)
    GPIO.output (LedB_2, GPIO.HIGH)
    GPIO.output (LedB_3, GPIO.HIGH)


def iniciarLeds():
    rospy.init_node('nodoLeds', anonymous=True)  # inicializa el nodo
    setPins()
    rospy.Subscriber('estado', Int32, handle_color)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


def handle_color(estado_act):
    if estado_act.data == 0:
        GPIO.output(LedA_1, GPIO.LOW)
        GPIO.output(LedA_2, GPIO.HIGH)
        GPIO.output(LedA_3, GPIO.HIGH)
        GPIO.output(LedB_1, GPIO.HIGH)
        GPIO.output(LedB_2, GPIO.HIGH)
        GPIO.output(LedB_3, GPIO.HIGH)
    elif estado_act.data == 1:
        GPIO.output(LedA_1, GPIO.HIGH)
        GPIO.output(LedA_2, GPIO.LOW)
        GPIO.output(LedA_3, GPIO.HIGH)
        GPIO.output(LedB_1, GPIO.HIGH)
        GPIO.output(LedB_2, GPIO.HIGH)
        GPIO.output(LedB_3, GPIO.HIGH)
    elif estado_act.data == 2:
        GPIO.output(LedA_1, GPIO.HIGH)
        GPIO.output(LedA_2, GPIO.HIGH)
        GPIO.output(LedA_3, GPIO.LOW)
        GPIO.output(LedB_1, GPIO.HIGH)
        GPIO.output(LedB_2, GPIO.HIGH)
        GPIO.output(LedB_3, GPIO.HIGH)
    elif estado_act.data == 3:
        GPIO.output(LedA_1, GPIO.HIGH)
        GPIO.output(LedA_2, GPIO.HIGH)
        GPIO.output(LedA_3, GPIO.HIGH)
        GPIO.output(LedB_1, GPIO.LOW)
        GPIO.output(LedB_2, GPIO.HIGH)
        GPIO.output(LedB_3, GPIO.HIGH)
    elif estado_act.data == 4:
        GPIO.output(LedA_1, GPIO.HIGH)
        GPIO.output(LedA_2, GPIO.HIGH)
        GPIO.output(LedA_3, GPIO.HIGH)
        GPIO.output(LedB_1, GPIO.HIGH)
        GPIO.output(LedB_2, GPIO.LOW)
        GPIO.output(LedB_3, GPIO.HIGH)
    elif estado_act.data == 5:
        GPIO.output(LedA_1, GPIO.HIGH)
        GPIO.output(LedA_2, GPIO.HIGH)
        GPIO.output(LedA_3, GPIO.HIGH)
        GPIO.output(LedB_1, GPIO.HIGH)
        GPIO.output(LedB_2, GPIO.HIGH)
        GPIO.output(LedB_3, GPIO.LOW)


if __name__ == '__main__':
    try:
        iniciarLeds()
    except rospy.ROSInterruptException:
        pass
