#!/usr/bin/env python
# ********************************************************
#   LABORATORIO DE BIOROBOTICA 2018
#   Daniel Garces Marin
#   PROYECTO <TEPORINGO> :: Xochitonal
#
#       Nodo disenado especificamente para recibir comandos de direccion y velocidad (en caso de que se requiera) para que,
#		usando los puertos GPIO de la Raspberry se comunique al shield de arduino para el control de los motores.
#       ->>Versión modificada para la prueba de motores (se mantienen los mensajes de salida a pantalla)
#
#   Ultima version:  17 de Septiembre del 2018
#   
#>>>>>>>> DaGaMa_ju-san <<<<<<<<<<<<<
# ***************************************************"""

######################################
#       >>>_DATASHEET_<<<
#
#       HEADER  GPIO           SHIELD
#          02    --             Vdd
#          04    --      Vin
#          11     17            M1-A
#          12     18            PWM-M1
#          13     27            M1-B
#          16     23            PWM-M2
#          18     24            M2-A
#          22     25            M2-B
#
######################################

#LIBRERIAS
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
#from base_node.srv import *
import RPi.GPIO as GPIO
import time
#-------------------------------------------------------------------

#>>CONFIGURACIoN DE LOS PINES  ---- Configure GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,  GPIO.OUT) #M1-A
GPIO.setup(18,  GPIO.OUT) #PWM-1
GPIO.setup(23,  GPIO.OUT) #PWM-2
GPIO.setup(24,  GPIO.OUT) #M2-A
GPIO.setup(25,  GPIO.OUT) #M2-B
GPIO.setup(27,  GPIO.OUT) #M1-B
pwm_r = GPIO.PWM(18, 255) #PWM-1
pwm_l = GPIO.PWM(23, 255) #PWM-2
pwm_r.start(0)
pwm_l.start(0)
#----------------------------------------------------------------------

##>>>>>FUNCIONES

def forward(vel):
        print("AVANZA")
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(25,False)
        GPIO.output(24,True)
        GPIO.output(27,True)
        GPIO.output(17,False)
        pwm_l.ChangeDutyCycle(vel)
        pwm_r.ChangeDutyCycle(vel)

def backward(vel):
        print("RETROCEDE")
        #vel=20
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(25,True)
        GPIO.output(24,False)
        GPIO.output(27,False)
        GPIO.output(17,True)
        pwm_l.ChangeDutyCycle(vel)
        pwm_r.ChangeDutyCycle(vel)

def left(vel):
        print("GIRO_IZQUIERDA")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(25,False)
        GPIO.output(24,True)
        GPIO.output(27,False)
        GPIO.output(17,True)
        pwm_l.ChangeDutyCycle(vel)
        pwm_r.ChangeDutyCycle(vel)

def right(vel):
        print("GIRO_DERECHA")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(25,True)
        GPIO.output(24,False)
        GPIO.output(27,True)
        GPIO.output(17,False)
        pwm_l.ChangeDutyCycle(vel)
        pwm_r.ChangeDutyCycle(vel)

def turn_Fr(velI,velD):
        print("MOV_DERECHA AVANZANDO")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(25,True)
        GPIO.output(24,False)
        GPIO.output(27,True)
        GPIO.output(17,False)
        pwm_l.ChangeDutyCycle(velI)
        pwm_r.ChangeDutyCycle(velD)

def turn_Fl(velI,velD):
        print("MOV_IZQUIERDA AVANZANDO")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(25,False)
        GPIO.output(24,True)
        GPIO.output(27,False)
        GPIO.output(17,True)
        pwm_l.ChangeDutyCycle(velI)
        pwm_r.ChangeDutyCycle(velD)

def turn_Br(velI,velD):
        print("MOV_DERECHA RETROCEDIENDO")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(24,False)
        GPIO.output(25,True)
        GPIO.output(17,True)
        GPIO.output(27,False)
        pwm_l.ChangeDutyCycle(velI)
        pwm_r.ChangeDutyCycle(velD)

def turn_Bl(velI,velD):
        print("MOV_IZQUIERDA RETROCEDIENDO")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(24,False)
        GPIO.output(25,True)
        GPIO.output(17,True)
        GPIO.output(27,False)
        pwm_l.ChangeDutyCycle(velI)
        pwm_r.ChangeDutyCycle(velD)

def stop():
        print("ALTO")
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
#----------------------------------------------------------------------------------------------------------------

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ">>>COMANDO PROPORCIONADO:_ %s", data.data)
    #Datos arrojados por el topico tipo string::

    vel_giro=80; ##Velocidad predefinida para los movimientos del teporingo

    #Definicion de tiempos para el control de los movimientos del robot
    #time_avance=0.84 #Resagada
    time_avance=0.35
    time_transicion=0.01
    time_giro=0.5

    #Intruccion enviada por el topico
    orden=str(data.data) #Obteniendo el valor del topico

    if orden=='Fw': #ADELANTE
        forward(vel_giro)
        time.sleep(time_avance)
        #backward(vel_tepo)
        #time.sleep(time_transicion)
        #stop()

    elif orden=='Bw':#ATRAS
        backward(vel_giro)
        time.sleep(time_avance)
        #forward(vel_tepo)
        #time.sleep(time_transicion)
        #stop()

    elif orden=="R": #Giro_Derecha
        right(vel_giro)
        time.sleep(time_giro)
        #stop()

    elif orden=='L': #Giro_Izquierda
        left(vel_giro)
        time.sleep(time_giro)
        #stop()

    elif orden=='FwR': #Giro_Hacia_Derecha
        vel_tepoI=vel_giro;
        vel_tepoD=vel_giro/2;
        turn_Fr(vel_tepoD,vel_tepoI)
        time.sleep(time_avance)
        stop()

    elif orden=='FwL': #Giro_Hacia_Derecha
        vel_tepoI=vel_giro/2;
        vel_tepoD=vel_giro;
        turn_Fl(vel_tepoD,vel_tepoI)
        time.sleep(time_avance)
        stop()

    elif orden=='BwR': #Giro_Hacia_Derecha
        vel_tepoI=vel_giro;
        vel_tepoD=vel_giro/2;
        turn_Br(vel_tepoD,vel_tepoI)
        time.sleep(time_avance)
        stop()

    elif orden=='BwL': #Giro_Hacia_Derecha
        vel_tepoI=vel_giro/2;
        vel_tepoD=vel_giro;
        turn_Bl(vel_tepoD,vel_tepoI)
        time.sleep(time_avance)
        stop()

    elif orden=='stop': #ALTO
        stop()

    else:
        stop()

#----------------------------------------------------------------------------------------------------------------

def listener():

    rospy.init_node('listener', anonymous=True)
    print("_>motor_node para pruebas en linea")
    rospy.Subscriber("/hardware/motor/direction",String, callback) #Nuevo topico a considerar
    #Datos arrojados por el topico: float32MultiArray :: Rango=[-0.5,0.5] Formato=[VL,VR]

    # Ejectuta ROS hasta que se termina el servicio
    rospy.spin()
#____________________________________________________________________________________________________________________

if __name__ == '__main__':
    listener()
