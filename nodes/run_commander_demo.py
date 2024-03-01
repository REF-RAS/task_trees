#!/usr/bin/env python3

# Copyright 2023 - Andrew Kwok Fai LUI, Centre for Robotics
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2023'
__license__ = 'GPL'
__version__ = '0.0.1'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

# general modules
import os, json, random, signal, copy, glob, collections, sys, time, yaml, argparse
# sys.path.append('/home/qcr/robotarchi_moveit_ws/devel/lib/python3/dist-packages')
import rospy, actionlib
import message_filters
from std_msgs.msg import String
# project modules
from examples.commander_demo import GeneralCommanderDemo 

# ----- the main program
if __name__ == '__main__':
    """ A script for running the General Commander Demonstration
    """
    # rospy.init_node('run_commander_demo', anonymous=False)
    try:
        rospy.loginfo('commander demo is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)