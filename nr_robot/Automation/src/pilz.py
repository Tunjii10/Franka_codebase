#!/usr/bin/env python3
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("thermo_python", anonymous=True)

    robot = moveit_commander.RobotCommander()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
if __name__ == "__main__":
    main()
