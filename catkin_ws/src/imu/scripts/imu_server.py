#!/usr/bin/env python
import rospy
from imu.srv import *
from std_msgs.msg import Float32MultiArray

import RTIMU
import os.path
import math
import time

global imu

res=Float32MultiArray()


SETTINGS_FILE = "RTIMULib_LSM9DS1"

imu_conected=True

global data

def degree_convertion(arr):
	arr=list(arr)
	for i in range(len(arr)):
		arr[i]=math.degrees(arr[i])

	return arr

def to_360_deg(arr):
	arr=list(arr)
	for i in range(len(arr)):
		if arr[i]<0:
			arr[i]=2*math.pi+arr[i]
	return arr

def handle_imu(req):
	global imu
	global data

	if req.readings=="fusion_pose":
		print "fusion_pose"
		res.data=data["fusionPose"]
		if req.degrees:
			res.data=degree_convertion(res.data)
	elif req.readings=="compass":
		print "compass"
		res.data=data["compass"]
	elif req.readings=="accel":
		print "accel"
		res.data=data["accel"]
	elif req.readings=="gyro":
		print "gyro"
		res.data=data["gyro"]
	elif req.readings=="all":
		print "all"
		res.data=data["accel"]+data["gyro"]+data["compass"]
	elif req.readings=="fusion_pose_360":
		print "fusion_pose_360"
		res.data=to_360_deg(data["fusionPose"])
		if req.degrees:
			res.data=degree_convertion(res.data)
	else:
		print "not a valid string"
		res.data=[]

	return ImuValuesResponse(res)

def main():
	global imu
	global data

	print("Initializig imu server")
	rospy.init_node('ImuValuesServer')
	s= rospy.Service('imu',ImuValues, handle_imu)

	print("Using settings file " + SETTINGS_FILE + ".ini")
	if not os.path.exists(SETTINGS_FILE + ".ini"):
	  print("Settings file does not exist, will be created")

	s = RTIMU.Settings(SETTINGS_FILE)
	imu = RTIMU.RTIMU(s)

	print("IMU Name: " + imu.IMUName())

	if (not imu.IMUInit()):
	    print("IMU Init Failed")
	    imu_conected=False
	else:
	    print("IMU Init Succeeded")

	# this is a good time to set any fusion parameters

	imu.setSlerpPower(0.02)
	imu.setGyroEnable(True)
	imu.setAccelEnable(True)
	imu.setCompassEnable(True)

	poll_interval = imu.IMUGetPollInterval()
	print("Recommended Poll Interval: %dmS\n" % poll_interval)
	poll_interval=poll_interval*1.0/1000.0
	print "reading"
	if imu.IMURead():
		data = imu.getIMUData()
	time.sleep(poll_interval)
	rospy.spin()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass