#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from automation.srv import *
import tf
import numpy as np

def movePoseClient():
    # Wait for service 
    rospy.wait_for_service('impedance_pose') 

    try:
        # position arguments for target object
        posX = 0.6
        posY = -0.0311708152294
        posZ = 0.05
        ortX = 0.999606072903
        ortY = -0.00390968192369
        ortZ = 0.0277839880437
        ortW = 0.000708106090315
       
        # initialise service proxy    
        move_Pose = rospy.ServiceProxy('impedance_pose', movePose)
        resp = move_Pose(posX, posY, posZ, ortX, ortY, ortZ, ortW)
        # print service call response to screen
        return print(resp.result)
    except rospy.ServiceException as e:
        print("The Service call failed: %s" % e)


if __name__ == "__main__":
    
    movePoseClient()
