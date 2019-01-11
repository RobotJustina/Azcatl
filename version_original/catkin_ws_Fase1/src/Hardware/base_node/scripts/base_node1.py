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
        vel=21.5
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
        vel=21.5
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
        vel=25.7
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
        vel=25.7
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


def handle_over(req):

        ovr = req.over_flg
        print "Handle Over flg ",str(ovr)

        if (ovr):
                #ROS_INFO("base_node received over signal")
                flg_read=1
                flg_ENVIRONMENT=1
                req.over_flg = 0

        print "Returning ",str(req.over_flg)
#        return MVServResponse(req.over_flg)



def handle_mv_service(req):
    mv=req.param.split()
    print "Base module receiving [%s]"%(req.param)
	
    if (mv[0] == "mv"):
    	dist=float(mv[1])
	ang=float(mv[2])
	if (ang > 0):
	        left()
        else:
                ang=ang*-1
                right()
        if (ang < 1):
                time.sleep(0.45*ang) #45 
        elif (ang < 1.74):
                time.sleep(0.37*ang) #90
        else: 
                time.sleep(0.31*ang) #>90
        
        stop()
        

	if (dist > 0):
		forward()
		
	else:
		dist=dist*-1
		backward()
	if (dist < 0.2):
		time.sleep(2.55*dist) #10 cm
	elif (dist < 0.4):
		time.sleep(1.62*dist) #30cm
	elif (dist < 0.6):
                time.sleep(1.4*dist) #50cm
	else:
		time.sleep(1.17*dist) #>50cm
	stop()			
		
    else:
		print "Comando no reconocido"
		stop()	

    print "Returning ", req.param
    return MVServResponse(req.param)

def handle_position(req):

	#ss = range(2)

	x=req.coord_x
	y=req.coord_y
	angle=req.coord_ang

        print "Base module position x ",str(x)," y ",str(y)," angle ",str(angle)

        s="position x "+str(x)+" y "+str(y)+" angle "+str(angle)
        #time.sleep(delay)
	#ss[0] =  'dummy' 
	#ss[1] = s 
        print "Returning ",s
	return MVServResponse(s)



def mv_service():
    rospy.init_node('mv_server')
    s = rospy.Service('move_robot', MVServ, handle_mv_service)
    s = rospy.Service('send_position_mv', MVServ, handle_position)
    service_1 = rospy.Service("over_base",OverSrv, handle_over);




    print "Ready to move robot."
    rospy.spin()

if __name__ == "__main__":
    mv_service()
