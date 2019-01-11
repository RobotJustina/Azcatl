#!/usr/bin/env python
# ********************************************************
#   Laboratorio de Biorobotica
#   Daniel Garces Marin
#   Teporingo, Version SmartThings,2
#       Este nodo acepta ahora comandos de dirrecion en vez recibir las
#       velocidades de las llantas como en el anterior nodo
#   Ultima version: 10 de Junio del 2018
#   
# ***************************************************"""

#LIBRERIAS
#DaGaMa_ju-san
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
GPIO.setup(18,  GPIO.OUT)
GPIO.setup(23,  GPIO.OUT)
GPIO.setup(24,  GPIO.OUT)
GPIO.setup(25,  GPIO.OUT)
GPIO.setup(17,  GPIO.OUT)
GPIO.setup(27,  GPIO.OUT)
pwm_l = GPIO.PWM(18, 255)
pwm_r = GPIO.PWM(23, 255)
pwm_l.start(0)
pwm_r.start(0)
#----------------------------------------------------------------------
def forward(vel):
        print("Teporingo AVANZA")
        #vel=20
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(24,True)
        GPIO.output(25,False)
        GPIO.output(17,False)
        GPIO.output(27,True)
        pwm_l.ChangeDutyCycle(vel)
        pwm_r.ChangeDutyCycle(vel)

def backward(vel):
        print("Teporingo RETROCEDE")
        #vel=20
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(24,False)
        GPIO.output(25,True)
        GPIO.output(17,True)
        GPIO.output(27,False)
        pwm_l.ChangeDutyCycle(vel)
        pwm_r.ChangeDutyCycle(vel)

def right(vel):
        print("Teporingo GIRO_IZQUIERDA")
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

def left(vel):
        print("Teporingo GIRO_DERECHA")
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

def turn_Fr(velI,velD):
        print("Teporingo MOV_DERECHA AVANZANDO")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(24,True)
        GPIO.output(25,False)
        GPIO.output(17,False)
        GPIO.output(27,True)
        pwm_l.ChangeDutyCycle(velI)
        pwm_r.ChangeDutyCycle(velD)

def turn_Fl(velI,velD):
        print("Teporingo MOV_IZQUIERDA AVANZANDO")
        #vel=30
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        time.sleep(0.1)
        GPIO.output(24,True)
        GPIO.output(25,False)
        GPIO.output(17,False)
        GPIO.output(27,True)
        pwm_l.ChangeDutyCycle(velI)
        pwm_r.ChangeDutyCycle(velD)

def turn_Br(velI,velD):
        print("Teporingo MOV_DERECHA RETROCEDIENDO")
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
        print("Teporingo MOV_IZQUIERDA RETROCEDIENDO")
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
        print("Teporingo ALTO")
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
#----------------------------------------------------------------------------------------------------------------

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ">>>COMANDO PROPORCIONADO:_ %s", data.data)
    #Datos arrojados por el topico tipo string::

    vel_tepo=30; ##Velocidad predefinida para los movimientos del teporingo
    time_avance=0.84
    time_transicion=0.01
    vel_giro=30
    time_giro=0.5
    orden=str(data.data) #Obteniendo el valor del topico


    if orden=='Fw': #ADELANTE
        forward(vel_tepo)
        time.sleep(time_avance)
        backward(vel_tepo)
        time.sleep(time_transicion)
        stop()

    elif orden=='Bw':#ATRAS
        backward(vel_tepo)
        time.sleep(time_avance)
        forward(vel_tepo)
        time.sleep(time_transicion)
        stop()

    elif orden=="R": #Giro_Derecha
        right(vel_giro)
        time.sleep(time_giro)
        stop()

    elif orden=='L': #Giro_Izquierda
        left(vel_giro)
        time.sleep(time_giro)
        stop()

    elif orden=='FwR': #Giro_Hacia_Derecha
        vel_tepoI=vel_tepo;
        vel_tepoD=vel_tepo/2;
        turn_Fr(vel_tepoD,vel_tepoI)
        time.sleep(time_avance)
        stop()

    elif orden=='FwL': #Giro_Hacia_Derecha
        vel_tepoI=vel_tepo/2;
        vel_tepoD=vel_tepo;
        turn_Fl(vel_tepoD,vel_tepoI)
        time.sleep(time_avance)
        stop()

    elif orden=='BwR': #Giro_Hacia_Derecha
        vel_tepoI=vel_tepo;
        vel_tepoD=vel_tepo/2;
        turn_Br(vel_tepoD,vel_tepoI)
        time.sleep(time_avance)
        stop()

    elif orden=='BwL': #Giro_Hacia_Derecha
        vel_tepoI=vel_tepo/2;
        vel_tepoD=vel_tepo;
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
    print("Teporingo en linea")
    rospy.Subscriber("/hardware/mobile_base/direction", String, callback) #Nuevo topico a considerar
    #Datos arrojados por el topico: float32MultiArray :: Rango=[-0.5,0.5] Formato=[VL,VR]

    # Ejectuta ROS hasta que se termina el servicio
    rospy.spin()
#____________________________________________________________________________________________________________________

if __name__ == '__main__':
    listener()
