#!/usr/bin/env python3

# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import os, random
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector
# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
from task_trees_manager_rviz import *

if __name__=='__main__':
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_task_manager = RVizTaskTreesManager(arm_commander)

        logger.info('rviz_display demo is running')
        the_task_manager.spin()
    except Exception as e:
        logger.exception(e)
      


