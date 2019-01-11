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
        global pubLocation, pubDevice, pubSpeeds
        global newSpeeds, msgSpeeds
        global percentLinearVel, percentAngularVel
        global robotAvailable
        while True:
            time.sleep(0.02)
            commandReceived = str(SerialPort.readline()).strip()
            print "[Smart Things Node] >>> Command: " + commandReceived
            if(commandReceived == "stop"):
                if(not robotAvailable):
                    print "[Smart Things Node] >>> Robot is busy right now..."
                    continue
                newSpeeds = False
                msgSpeeds.data = [0.0, 0.0]
                pubSpeeds.publish(msgSpeeds)
            elif(commandReceived.find('-') == 3 and len(commandReceived) > 4):
                commandSplitted = commandReceived.split("-",1)
                if(commandSplitted[0] == "mov"):
                    if(not robotAvailable):
                        print "[Smart Things Node] >>> Robot is busy right now..."
                        continue
                    move(commandSplitted[1])
                elif(commandSplitted[0] == "dev"):
                    pubDevice.publish(commandSplitted[1])
                    newSpeeds = False
                elif(commandSplitted[0] == "loc"):
                    pubLocation.publish(commandSplitted[1])
                    newSpeeds = False
                elif(commandSplitted[0] == "lin"):
                    percentLinearVel = float(commandSplitted[1])/100
                    updateSpeeds()
                elif(commandSplitted[0] == "ang"):
                    percentAngularVel = float(commandSplitted[1])/100
                    updateSpeeds()

def move(direction):
    global newSpeeds
    if(direction == "FwL"):
        setCoefficients(1.0, -1.0, 1.0, 1.0)
    elif(direction == "Fw"):
        setCoefficients(1.0, 0.0, 1.0, 0.0)
    elif(direction == "FwR"):
        setCoefficients(1.0, 1.0, 1.0, -1.0)
    elif(direction == "L"):
        setCoefficients(0.0, -1.0, 0.0, 1.0)
    elif(direction == "R"):
        setCoefficients(0.0, 1.0, 0.0, -1.0)
    elif(direction == "BwL"):
        setCoefficients(-1.0, 1.0, -1.0, -1.0)
    elif(direction == "Bw"):
        setCoefficients(-1.0, 0.0, -1.0, 0.0)
    elif(direction == "BwR"):
        setCoefficients(-1.0, -1.0, -1.0, 1.0)
    else:
        return
    newSpeeds = True

def setCoefficients(c1, c2, c3, c4):
    global c1Left, c2Left #Coefficients left speed
    global c1Right, c2Right #Coefficients right speed
    c1Left = c1
    c2Left = c2
    c1Right = c3
    c2Right = c4
    updateSpeeds()

def updateSpeeds():
    global msgSpeeds
    global percentLinearVel, percentAngularVel
    global c1Left, c2Left #Coefficients left speed
    global c1Right, c2Right #Coefficients right speed
    L = 0.23 #Robot diameter [m]
    maxLinearVel = 0.25 #[m/s]
    maxAngularVel = 0.523 #[rad/s]
    v = percentLinearVel*maxLinearVel #Linear
    w = percentAngularVel*maxAngularVel #Angular
    msgSpeeds.data[0] = c1Left*v + c2Left*L/2.0*w #Left speed
    msgSpeeds.data[1] = c1Right*v + c2Right*L/2.0*w #Right speed

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
    global pubLocation, pubDevice, pubSpeeds
    rospy.init_node("smart_things_node")
    pubLocation = rospy.Publisher("/navigation/goal_location", String, queue_size = 1)
    pubDevice = rospy.Publisher("/navigation/smart_things/alerts_devices", String, queue_size = 1)
    pubSpeeds = rospy.Publisher("/hardware/mobile_base/speeds", Float32MultiArray, queue_size = 1)
    subBattery = rospy.Subscriber("/hardware/robot_state/base_battery", Float32, callbackBattery)
    subRobotAvailability = rospy.Subscriber("/hardware/robot_state/availability", Bool, callbackRobotAvailability)
    rate = rospy.Rate(30)
    #BatteryInfo
    global newBatteryInfo, batteryCharge
    newBatteryInfo = False
    batteryCharge = 0.0
    #Teleop Mode
    global percentLinearVel, percentAngularVel
    global c1Left, c2Left
    global c1Right, c2Right
    percentLinearVel = 1.0
    percentAngularVel = 1.0
    c1Left = 0.0
    c2Left = 0.0
    c1Right = 0.0
    c2Right = 0.0
    global msgSpeeds, newSpeeds
    newSpeeds = False
    msgSpeeds = Float32MultiArray()
    msgSpeeds.data = [0.0, 0.0]
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
        if newSpeeds:
            pubSpeeds.publish(msgSpeeds)
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
