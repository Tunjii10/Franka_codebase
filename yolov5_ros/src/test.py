#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from yolov5_ros.srv import petri, petriResponse


def petriClient():
    # Wait for service 
    rospy.wait_for_service('/petri_service') 
    try:
        # position arguments for target object
        detect = True
       
        # initialise service proxy    
        petri_Detect = rospy.ServiceProxy('/petri_service', petri)
        resp = petri_Detect(True)
        # print service call response to screen
        return print(resp.result)
    except rospy.ServiceException as e:
        print("The Service call failed: %s" % e)


if __name__ == "__main__":

    petriClient()