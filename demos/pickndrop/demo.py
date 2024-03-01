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

import operator, yaml, os, math, random, copy, sys, signal, threading
from enum import Enum
import rospy
import py_trees

# robot control module
from arm_commander.commander_moveit import GeneralCommander
from task_trees.states import COMPLETION_STATES
from demos.pickndrop.task_trees_manager_pnd import *
# -- Test cases specialized for the PickNDrop Demo

class DemoStates(Enum):
    INIT = 0  # generate the ball
    TO_SCAN = 1  # run the scan task
    SCAN = 2  # waiting for scan result
    PICK = 3  # run the pick up task
    DROP = 4  # run the drop task
    STOP = 5

class PNDTaskManagerDemo():
    """ Test cases specialized for the PickNDrop Demo
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='seen_object', access=py_trees.common.Access.READ)  
        
        self.arm_commander = GeneralCommander('panda_arm')
        self.the_task_manager = PNDTaskTreesManager(self.arm_commander)
        self.the_task_manager.display_tree()
        self.to_stop = False
        self._run_demo()
        rospy.spin()

    def stop(self, *args, **kwargs):
        rospy.loginfo('stop signal received')
        self.the_task_manager.shutdown()
        sys.exit(0)

    def _run_scan(self):
        task_manager = self.the_task_manager
        rospy.loginfo(f'=== Test 1')       
        
        rospy.loginfo(f'=== Submit a ScanTask')
        the_task = ScanTask()
        task_manager.submit_task(the_task)
        the_task.wait_for_completion()     

        rospy.loginfo(f'=== Test Finished')

    def _run_demo(self):
        task_manager = self.the_task_manager
        state = DemoStates.INIT
        the_task = None
        rospy.loginfo(f'=== PickNDrop Demo Started') 
         
        while not self.to_stop:
            rospy.sleep(0.1)
            if state == DemoStates.INIT:
                rospy.loginfo(f'=== PickNDrop INIT: Initialization of Pose and Submit Scan Task')
                task_manager.submit_task(the_task := ScanTask()) 
                state = DemoStates.SCAN
                rospy.loginfo(f'=== PickNDrop INIT: Waiting for the Scan Task to WORKING')
                the_task.wait_for_working()
                rospy.loginfo(f'=== PickNDrop INIT: The Scan Task is WORKING and Generate a New Target')
                self.the_task_manager.generate_a_ball()
            elif state == DemoStates.SCAN:
                self.the_task_manager.simulate_camera()
                if self.the_blackboard.exists('seen_object'):
                    xyz = self.the_blackboard.seen_object
                    rospy.loginfo(f'=== PickNDrop SCAN: An object Found at {xyz} and so Cancel the Scantask and Wait')
                    the_task.cancel(wait=True)
                    rospy.loginfo(f'=== PickNDrop SCAN: Submit PickUpObjectTask')                    
                    task_manager.submit_task(the_task := PickUpObjectTask(xyz_world=xyz))
                    state = DemoStates.PICK
            elif state == DemoStates.PICK:
                the_task.wait_for_completion()
                if the_task.get_state() not in [TaskStates.SUCCEEDED]:
                    rospy.loginfo(f'=== PickNDrop PICK: Pick Failed')
                    state = DemoStates.STOP
                else:
                    rospy.loginfo(f'=== PickNDrop PICK: Pick Successful and Submit Drop Task')
                    task_manager.submit_task(the_task := DropObjectTask()) 
                    state = DemoStates.DROP
            elif state == DemoStates.DROP:
                the_task.wait_for_completion() 
                rospy.loginfo(f'=== PickNDrop DROP: Drop Successful')
                state = DemoStates.INIT
            elif state == DemoStates.STOP:
                rospy.loginfo(f'=== PickNDrop STOP: Submit a MoveNamedPoseTask to stow and then stop')
                the_task = MoveNamedPoseTask('stow')
                task_manager.submit_task(the_task)
                the_task.wait_for_completion()       
                return           

if __name__=='__main__':
    rospy.init_node('run_pnd_task_manager_demo', anonymous=False)
    try:
        PNDTaskManagerDemo()
        rospy.loginfo('pnd_task_manager_demo is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


