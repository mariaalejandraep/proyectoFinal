#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time
import math
import sys
from std_msgs.msg import Float32MultiArray

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
cicloADriver = 0
# Ciclo util del pulso para el motor B. Un numero entre 0 y 100.
cicloBDriver = 0
# Variable con el pin que va del encoder con la senal A
pwmA1Encoder = 35
# Variable con el pin que va del encoder con la senal A
pwmB1Encoder = 36
# Variable con el pin que va del encoder con la senal A
pwmA2Encoder = 37
# Variable con el pin que va del encoder con la senal A
pwmB2Encoder = 38
# radio de la rueda
radioRueda = 14.65  # milimetros
# Variables que referencian senales PMW de los encoders
pA1 = None
pA2 = None
pB1 = None
pB2 = None
# Variables de conteo de flancos de encoders
contadorA = 0

# Referencia de ultimo contador de ruedas
refContadorA = 0
refTiempoA = 0

refAccionControlA = 0

pwmA = 0

subidaA2 = []
subidaB2 = []
# Variable booleana  que inidca si se esta calculando velocidad
calculando = False
# Radio de las llantas en metros
r = (29.3/2)
# Variables de control PI
kp = 0.0001
ki = 0
# Acumulacion de error para integrador
integradorA = []
integradorB = []
# Velocidades de referencia de las ruedas // ASUMIRE rueda A izquierda (vel positiva counter clockwise, B antes que A) y
# rueda B derecha (vel positiva clockwise, A antes que B)
velRefA = 0
velRefB = 0
# Velocidades actual de las ruedas
velActA = 0
velActB = 0
# Variable booleana que hace referencia a que se esta mivendo
movingA = False
movingB = False
# Variable de saturacion maxima de ciclo util
satCicloUtil = 35


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
    GPIO.output(pwmA1Driver, 0)
    pA2 = GPIO.PWM(pwmA2Driver, fDriver)
    GPIO.output(pwmA2Driver, 0)
    pB1 = GPIO.PWM(pwmB1Driver, fDriver)
    GPIO.output(pwmB1Driver, 0)
    pB2 = GPIO.PWM(pwmB2Driver, fDriver)
    GPIO.output(pwmB2Driver, 0)
    # Detectar flancos en otros metodos
    GPIO.add_event_detect(pwmA1Encoder, GPIO.BOTH, callback=sumarFlancoA1)
    GPIO.add_event_detect(pwmB1Encoder, GPIO.BOTH, callback=sumarFlancoB1)
    # GPIO.add_event_detect(pwmA2Encoder, GPIO.RISING, callback=sumarFlancoA2)
    # GPIO.add_event_detect(pwmB2Encoder, GPIO.RISING, callback=sumarFlancoB2)


def controlBajoNivel():
    rospy.init_node('controlEncoder', anonymous=True)
    rospy.Subscriber('velocidad_deseada', Float32MultiArray, handle_velocidad_deseada)
    rate = rospy.Rate(h)
    setPins()
    pA1.ChangeDutyCycle(cicloADriver)
    pB1.ChangeDutyCycle(cicloBDriver)
    while not rospy.is_shutdown():
        calcularVelocidadRuedas()
        aplicarControlBajoNivel()
        print(contadorA)
        rate.sleep()
    apagar()


def calcularVelocidadRuedas():
    global velActA, refContadorA, refTiempoA
    flancos = contadorA - refContadorA
    tiempoNuevo = float(time.time())
    tiempo = tiempoNuevo - refTiempoA
    refContadorA = contadorA
    refTiempoA = tiempoNuevo
    velActA = (flancos/tiempo)*(math.pi/600)*radioRueda
    print("La velocidad actual de la rueda A:", velActA)


def aplicarControlBajoNivel():
    global integradorA, integradorB, pA1, pA2, pB1, pB2, refAccionControlA, pwmA
    errorA = velRefA - velActA
    errorB = velRefB - velActB
    integradorA.append(errorA)
    integradorA = integradorA[-5:]
    integradorB.append(errorB)
    integradorB = integradorB[-5:]
    integralA = sum(integradorA)
    integralB = sum(integradorB)
    errorSignalA = kp * errorA + ki * integralA
    pwmA = pwmA + errorSignalA
    errorSignalB = kp * errorB + ki * integralB
    if pwmA >= 0:
        print("Entro if rueda A")
        if pwmA > satCicloUtil:
            pwmA = satCicloUtil
        if refAccionControlA < 0:
            pwmA = 0
        pA2.stop()
        GPIO.output (pwmA2Driver, 0)
        pA1.start (0)
        pA1.ChangeDutyCycle(pwmA)
    else:
        print ("Entro else rueda A")
        if pwmA < -satCicloUtil:
            pwmA = -satCicloUtil
        if refAccionControlA > 0:
            pwmA = 0
        pA1.stop ()
        GPIO.output (pwmA1Driver, 0)
        pA2.start (0)
        pA2.ChangeDutyCycle (abs(pwmA))
    # if errorSignalB >= 0:
    #     if errorSignalB > satCicloUtil:
    #         errorSignalB = satCicloUtil
    #     pB2.stop ()
    #     GPIO.output (pwmB2Driver, 0)
    #     pB1.start (0)
    #     pB1.ChangeDutyCycle (errorSignalB)
    # else:
    #     if errorSignalB < -satCicloUtil:
    #         errorSignalB = satCicloUtil
    #     errorSignalB = abs(errorSignalB)
    #     pB1.stop ()
    #     GPIO.output (pwmB1Driver, 0)
    #     pB2.start (0)
    #     pB2.ChangeDutyCycle (errorSignalB)
    refAccionControlA = pwmA
    print("Ciclo util rueda A:", pwmA)
    # print("Ciclo util rueda B:", errorSignalB)


def handle_velocidad_deseada(vel):
    global velRefA, velRefB
    velRefA = vel.data[0]
    velRefB = vel.data[1]


def apagar():
    global cicloADriver, cicloBDriver
    cicloADriver = 0
    cicloBDriver = 0
    GPIO.output(pwmA1Driver, 0)
    GPIO.output(pwmA2Driver, 0)
    GPIO.output(pwmB1Driver, 0)
    GPIO.output(pwmB2Driver, 0)
    GPIO.cleanup()
    rospy.loginfo("Apagando.")


def sumarFlancoA1(channel):
    global contadorA
    if GPIO.input(pwmA1Encoder):
        if GPIO.input(pwmB1Encoder):
            contadorA = contadorA + 1
        else:
            contadorA = contadorA - 1
    else:
        if GPIO.input(pwmB1Encoder):
            contadorA = contadorA - 1
        else:
            contadorA = contadorA + 1


def sumarFlancoB1(channel):
    global contadorA
    if GPIO.input(pwmB1Encoder):
        if GPIO.input(pwmA1Encoder):
            contadorA = contadorA - 1
        else:
            contadorA = contadorA + 1
    else:
        if GPIO.input(pwmA1Encoder):
            contadorA = contadorA + 1
        else:
            contadorA = contadorA - 1


def sumarFlancoA2(channel):
    global subidaA2, moving
    subidaA2.append(time.time())
    subidaA2 = subidaA2[-13:]
    moving = True


def sumarFlancoB2(channel):
    global subidaB2, moving
    subidaB2.append(time.time())
    subidaB2 = subidaB2[-13:]
    moving = True


if __name__ == '__main__':
    try:
        if len(sys.argv) == 3:
            try:
                velRefA = float(sys.argv[1])
                velRefB = float(sys.argv[2])
            except ValueError:
                pass
        controlBajoNivel()
    except rospy.ROSInterruptException:
        pass
