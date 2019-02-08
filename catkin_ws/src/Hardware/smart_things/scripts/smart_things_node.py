#!/usr/bin/env python
import sys
import serial, threading, time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

def readSerialPort():
        global SerialPort
        global pubLocation, pubDevice, pubDirection
        global robotAvailable
        while True:
            time.sleep(0.01)
            commandReceived = str(SerialPort.readline()).strip()
            print "[Smart Things Node] >>> Command: " + commandReceived
            if(commandReceived == "stop"):
                if(not robotAvailable):
                    print "[Smart Things Node] >>> Robot is busy right now..."
                    continue
                pubDirection.publish(commandReceived)
            elif(commandReceived.find('-') == 3 and len(commandReceived) > 4):
                commandSplitted = commandReceived.split("-",1)
                if(commandSplitted[0] == "mov"):
                    if(not robotAvailable):
                        print "[Smart Things Node] >>> Robot is busy right now..."
                        continue
                    pubDirection.publish(commandSplitted[1])
                elif(commandSplitted[0] == "dev"):
                    pubDevice.publish(commandSplitted[1])
                    newSpeeds = False
                elif(commandSplitted[0] == "loc"):
                    pubLocation.publish(commandSplitted[1])
                    newSpeeds = False

def callbackBattery(msg):
    global newBatteryInfo, batteryCharge
    batteryCharge = int(round(100*(msg.data-9.0)/3.6))
    newBatteryInfo = True

def callbackRobotAvailability(msg):
    global robotAvailable, robotState
    global newSpeeds, newRobotState
    robotAvailable = msg.data
    newRobotState = True
    if(not robotAvailable):
        newSpeeds = False
        robotState = "busy"
    else:
        robotState = "on standby"

def main(portName, portBaudrate):
    print "[Smart Things Node] >>> Initializing Smart Things node..."
    global pubLocation, pubDevice, pubDirection
    rospy.init_node("smart_things_node")
    pubLocation = rospy.Publisher("/navigation/goal_location", String, queue_size = 1)
    pubDevice = rospy.Publisher("/navigation/smart_things/alerts_devices", String, queue_size = 1)
    pubDirection = rospy.Publisher("/hardware/smartThings/dir", String, queue_size = 1)
    subBattery = rospy.Subscriber("/hardware/robot_state/base_battery", Float32, callbackBattery)
    subRobotAvailability = rospy.Subscriber("/hardware/robot_state/availability", Bool, callbackRobotAvailability)
    rate = rospy.Rate(30)
    #BatteryInfo
    global newBatteryInfo, batteryCharge
    newBatteryInfo = False
    batteryCharge = 0.0
    #Availability
    global robotAvailable, newRobotState, robotState
    robotAvailable = True
    newRobotState = False
    robotState = ""
    #Serial Port
    global SerialPort
    print "[Smart Things Node] >>> Trying to open serial port \"" + portName + "\""
    SerialPort = serial.Serial(portName, portBaudrate)
    #Thread
    hilo = threading.Thread(target=readSerialPort)
    hilo.daemon = True
    hilo.start()
    while not rospy.is_shutdown():
        if newBatteryInfo:
            newBatteryInfo = False
            SerialPort.write("bL-" + str(batteryCharge))
            time.sleep(0.01)
        if newRobotState:
            newRobotState = False
            if(robotState == "busy"):
                SerialPort.write("rS-busy")
            else:
                SerialPort.write("rS-onSB")
            time.sleep(0.01)
            robotState = ""
        rate.sleep()
    if SerialPort.isOpen():
        SerialPort.close()
        print "[Smart Things Node] >>> Serial port closed..."

if __name__ == '__main__':
    #portName = "/dev/ArduinoUnoST"
    portName = "/dev/ttyACM0"
    portBaudrate = 115200
    try:
        if "--port" in sys.argv:
            portName = sys.argv[sys.argv.index("--port") + 1]
        main(portName, portBaudrate)
    except rospy.ROSInterruptException:
        pass
