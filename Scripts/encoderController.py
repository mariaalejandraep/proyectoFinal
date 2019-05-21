#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time, math

# Es la tasa en Hertz (Hz) del nodo.
h = 10
# Variable con el primer pin que va al driver para controlar el motor A.
pwmA1Driver = 11
# Variable con el segundo pin que va al driver para controlar el motor A.
pwmA2Driver = 12
# Variable con el primer pin que va al driver para controlar el motor B.
pwmB1Driver = 15
# Variable con el segundo pin que va al driver para controlar el motor B.
pwmB2Driver = 16
# Frecuencia en Hertz (Hz) de la senal de pulso que controla el motor.
fDriver = 500
# Ciclo util del pulso para el motor A. Un numero entre 0 y 100.
cicloADriver = 10
# Ciclo util del pulso para el motor B. Un numero entre 0 y 100.
cicloBDriver = 10
# Variable con el pin que va del encoder con la senal A
pwmA1Encoder = 35
# Variable con el pin que va del encoder con la senal A
pwmB1Encoder = 36
# Variable con el pin que va del encoder con la senal A
pwmA2Encoder = 37
# Variable con el pin que va del encoder con la senal A
pwmB2Encoder = 38
# Varibles que referencian senales PMW de los encoders
pA1 = None
pA2 = None
pB1 = None
pB2 = None
# Varibles de conteo de flancos de encoders
subidaA1 = []
subidaB1 = []
subidaA2 = []
subidaB2 = []
# Varible booleana  que inidca si se esta calculando velocidad
calculando = False
# Radio de las llantas en metros
r = (23.3/2000)


def setPins():
    global pA1, pA2, pB1, pB2
    # Configurandp estructura de pins de raspberry
    GPIO.setmode(GPIO.BOARD)
    # Configurando los pines de salida para el driver
    GPIO.setup(pwmA1Driver, GPIO.OUT)
    GPIO.setup(pwmA2Driver, GPIO.OUT)
    GPIO.setup(pwmB1Driver, GPIO.OUT)
    GPIO.setup(pwmB2Driver, GPIO.OUT)
    # Configurando pines de salida para los encoders
    GPIO.setup(pwmA1Encoder, GPIO.IN)
    GPIO.setup(pwmB1Encoder, GPIO.IN)
    GPIO.setup(pwmA2Encoder, GPIO.IN)
    GPIO.setup(pwmB2Encoder, GPIO.IN)
    # Configurando senales de salida para el driver e inicializandolas en ciclo util de 0
    pA1 = GPIO.PWM(pwmA1Driver, fDriver)
    pA1.start(0)
    pA2 = GPIO.PWM(pwmA2Driver, fDriver)
    pA2.start(0)
    pB1 = GPIO.PWM(pwmB1Driver, fDriver)
    pB1.start(0)
    pB2 = GPIO.PWM(pwmB2Driver, fDriver)
    pB2.start(0)
    # Detectar flancos en otros metodos
    GPIO.add_event_detect(pwmA1Encoder, GPIO.RISING, callback=sumarFlancoA1)
    GPIO.add_event_detect(pwmB1Encoder, GPIO.RISING, callback=sumarFlancoB1)
    GPIO.add_event_detect(pwmA2Encoder, GPIO.RISING, callback=sumarFlancoA2)
    GPIO.add_event_detect(pwmB2Encoder, GPIO.RISING, callback=sumarFlancoB2)


def controlBajoNivel():
    rospy.init_node('Raspberry_controller', anonymous=True)
    rate = rospy.Rate(h)
    setPins()
    pA1.ChangeDutyCycle(cicloADriver)
    pB1.ChangeDutyCycle(cicloBDriver)
    while not rospy.is_shutdown():
        rate.sleep()
        if subidaA1[len(subidaA1)-1] < subidaB1[len(subidaB1)-1]:
            print("Rueda 1 en dirreccion manecillas")
        else:
            print("Rueda 1 en dirreccion contra manecillas")
        if len(subidaA1) == 7:
            print("La velocidad de la rueda 1 es:", (2 * math.pi() * r) / (subidaA1[len(subidaA1) - 1] - subidaA1[0]))
        if subidaA2[len(subidaA2)-1] < subidaB2[len(subidaB2)-1]:
            print("Rueda 2 en dirreccion manecillas")
        else:
            print("Rueda 2 en dirreccion contra manecillas")
        if len(subidaA2) == 7:
            print("La velocidad de la rueda 2 es:", (2 * math.pi() * r) / (subidaA2[len(subidaA2) - 1] - subidaA2[0]))
    apagar()


def apagar():
    global cicloADriver, cicloBDriver
    cicloADriver = 0
    cicloBDriver = 0
    GPIO.output(pwmA1Driver, 0)
    GPIO.output(pwmA2Driver, 0)
    GPIO.output(pwmB1Driver, 0)
    GPIO.output(pwmB2Driver, 0)

    rospy.loginfo("Apagando.")


def sumarFlancoA1():
    global subidaA1
    subidaA1.append(time.time())
    subidaA1 = subidaA1[-7:]
    print(len(subidaA1))


def sumarFlancoB1():
    global subidaB1
    subidaB1.append(time.time())
    subidaB1 = subidaB1[-7:]
    # print(len(subidaB1))


def sumarFlancoA2():
    global subidaA2
    subidaA2.append(time.time())
    subidaA2 = subidaA2[-7:]
    # print(len(subidaA2))


def sumarFlancoB2():
    global subidaB2
    subidaB2.append(time.time())
    subidaB2 = subidaB2[-7:]
    # print(len(subidaB2))


if __name__ == '__main__':
    try:
        ken=2
    except rospy.ROSInterruptException:
        pass
