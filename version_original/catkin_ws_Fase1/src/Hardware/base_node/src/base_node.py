#!/usr/bin/env python
import RPi.GPIO as GPIO

from base_node.srv import *
import rospy
import time

#Configure GPIO
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

def forward():
        vel=20
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


def backward():
        vel=20
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


def left():
        vel=30
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

def right():
        vel=30
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

def stop():
        global pwm_l
        global pwm_r
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)

def handle_mv_service(req):
    mv=req.param.split()
    if (mv[0] == "mv"):
    	dist=float(mv[1])
	ang=float(mv[2])
	if (ang > 0):
	        left()
                time.sleep(0.4879592341*ang)
                stop()

        else:
                ang=ang*-1
                right()
                time.sleep(0.4879592341*ang)
                stop()

	if (dist > 0):
		forward()
		#if (dist < 20):
                #        time.sleep(2.15*dist) #10 cm
                #elif (dist < 40):
                #        time.sleep(1.72*dist) #-30cm
                #else:
                #time.sleep(1.4*dist) #50cm
		time.sleep(0.97*dist) #50cm
		#.sleep(0.95*dist) 2M
		stop()
	else:
		dist=dist*-1
		backward()
		#if (dist < 20):
		#	time.sleep(2.15*dist) #10 cm
		#elif (dist < 40):
		#	time.sleep(1.72*dist) #-30cm
		#else:
		#	time.sleep(1.4*dist) #50cm
		time.sleep(0.94*dist) #50cm
		stop()			
		
    else:
		print "Comando no reconocido"
		stop()	

    print req.param
    return MVServResponse(req.param)

def mv_service():
    rospy.init_node('base_node')
    s = rospy.Service('MVServ', MVServ, handle_mv_service)
    print "Ready to move robot."
    rospy.spin()

if __name__ == "__main__":
    mv_service()
