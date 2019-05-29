#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

# Ciclo util color 1 led A
cicloA1=100
# Ciclo util color 2 led A
cicloA2=100
# Ciclo util color 3 led A
cicloA3=100
# Ciclo util color 1 led B
cicloB1=100
# Ciclo util color 2 led B
cicloB2=100
# Ciclo util color 3 led B
cicloB3=100

# Variable que controla color 1 del LED A
LedA_1 =33
# Variable que controla color 2 del LED A
LedA_2 =32
# Variable que controla color 3 del LED A
LedA_3 =31
# Variable que controla color 1 del LED B
LedB_1 =29
# Variable que controla color 2 del LED B
LedB_2 =26
# Variable que controla color 3 del LED B
LedB_3 =24

#Frecuencia Leds
fLeds=500

def setPins():
    global pLedA_1,pLedA_2,pLedA_3,pLedB_1,pLedB_2,pLedB_3
    GPIO.setmode(GPIO.BOARD)
    # Configurando los pines de salida para LEDs
    GPIO.setup(LedA_1, GPIO.OUT)
    GPIO.setup(LedA_2, GPIO.OUT)
    GPIO.setup(LedA_3, GPIO.OUT)
    GPIO.setup(LedB_1, GPIO.OUT)
    GPIO.setup(LedB_2, GPIO.OUT)
    GPIO.setup(LedB_3, GPIO.OUT)

    # Configurando senales de salida para los LEDS e inicializandolas en ciclo util de 0
    pLedA_1 = GPIO.PWM(LedA_1, fLeds)
    GPIO.output(LedA_1, 100)
    pLedA_2 = GPIO.PWM(LedA_2, fLeds)
    GPIO.output(LedA_2, 100)
    pLedA_3 = GPIO.PWM(LedA_3, fLeds)
    GPIO.output(LedA_3, 100)
    pLedB_1 = GPIO.PWM(LedB_1, fLeds)
    GPIO.output(LedB_1, 100)
    pLedB_2 = GPIO.PWM(LedB_2, fLeds)
    GPIO.output(LedB_2, 100)
    pLedB_3 = GPIO.PWM(LedB_3, fLeds)
    GPIO.output(LedB_3, 100)



def iniciarLeds():
    rospy.init_node('nodoLeds', anonymous=True)  # inicializa el nodo
    setPins()
    rospy.Subscriber('estado', Int32, handle_color)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()



def handle_color(estado_act):
    global pLedA_1,pLedA_2,pLedA_3,pLedB_1,pLedB_2,pLedB_3,cicloA1,cicloA2,cicloA3,cicloB1,cicloB2,cicloB3
    if estado_act.data==0:
        cicloA1=0
        pLedA_1.ChangeDutyCycle(cicloA1)
    elif estado_act.data==1:
        cicloA1=100
        pLedA_1.ChangeDutyCycle(cicloA1)
        cicloA2=0
        pLedA_2.ChangeDutyCycle(cicloA2)
    elif estado_act.data==2:
        cicloA2=100
        pLedA_2.ChangeDutyCycle(cicloA2)
        cicloA3=0
        pLedA_3.ChangeDutyCycle(cicloA3)
    elif estado_act.data==3:
        cicloA3=100
        pLedA_3.ChangeDutyCycle(cicloA3)
        cicloB1=0
        pLedB_1.ChangeDutyCycle(cicloB1)
    elif estado_act.data==4:
        cicloB1=100
        pLedB_1.ChangeDutyCycle(cicloB1)
        cicloB2=0
        pLedB_2.ChangeDutyCycle(cicloB2)
    elif estado_act.data==5:
        cicloB2=100
        pLedB_2.ChangeDutyCycle(cicloB2)
        cicloB3=0
        pLedB_3.ChangeDutyCycle(cicloB3)



if __name__ == '__main__':
    try:
        iniciarLeds()
    except rospy.ROSInterruptException:
        pass
