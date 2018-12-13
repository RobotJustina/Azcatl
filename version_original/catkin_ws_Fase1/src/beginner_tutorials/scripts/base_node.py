#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_mv_service(req):
    print "Returning " + req.string
    return MVServResponse(req.string)

def mv_service():
    rospy.init_node('base_node')
    s = rospy.Service('mv_service', MVServ, handle_mv_service)
    print "Ready to move robot."
    rospy.spin()

if __name__ == "__main__":
    mv_service()
