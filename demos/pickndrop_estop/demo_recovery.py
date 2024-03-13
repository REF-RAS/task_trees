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

import operator, yaml, os, math, random, copy, sys, signal, threading, time
from enum import Enum
import rospy
import py_trees
from std_msgs.msg import Bool

# robot control module
from arm_commander.commander_moveit import GeneralCommander
from task_trees.states import COMPLETION_STATES
from task_trees_manager_pnd import *
# -- Test cases specialized for the PickNDrop Demo

class DemoStates(Enum):
    INIT = 0  # generate the ball
    TO_SCAN = 1  # run the scan task
    SCAN = 2  # waiting for scan result
    PICK = 3  # run the pick up task
    DROP = 4  # run the drop task
    STOP = 5

class GuardedPNDDemoApplication():
    """ The application program for the PickNDrop Demo with support for an EStop button and a recovery routine
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        # set up a topic subscriber at /estop to update the estop status 
        self.estop_status = False
        self.estop_topic_sub = rospy.Subscriber('/estop', Bool, self._cb_estop_update)
        # set up a blackboard variable for tracking if an object is attached
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='attached_object', access=py_trees.common.Access.WRITE)          
        self.the_blackboard.attached_object = False
        # set up the blackboard variable for recording 
        self.the_blackboard.register_key(key='seen_object', access=py_trees.common.Access.READ)  
        # set up the commander and the task manager
        self.arm_commander = GeneralCommander('panda_arm')
        self.the_task_manager = PNDTaskTreesManager(self.arm_commander)
        # add global guard condition function to the task manager
        self.the_task_manager.set_global_guard_condition_fn(self.estop_guard_condition)
        
        self.to_stop = False
        self._run_demo()
        rospy.spin()

    def stop(self, *args, **kwargs):
        rospy.loginfo('stop signal received')
        self.the_task_manager.shutdown()
        sys.exit(0)
        
    def estop_guard_condition(self):
        return not self.estop_status
        
    def _cb_estop_update(self, msg:Bool):
        self.estop_status = msg.data

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
        scene_setup = False
        while not self.to_stop:
            rospy.sleep(0.1)
            if state == DemoStates.INIT:
                rospy.loginfo(f'=== PickNDrop INIT: Initialization of Pose and Submit Scan Task')
                task_manager.submit_task(the_task := ScanTask()) 
                state = DemoStates.SCAN
                rospy.loginfo(f'=== PickNDrop INIT: Waiting for the Scan Task to WORKING')
                the_task.wait_for_working()
                rospy.loginfo(f'=== PickNDrop INIT: The Scan Task is WORKING, setup the scene if needed and generate a new target')
                if not scene_setup:
                    task_manager.setup_objects()
                    scene_setup = True
                self.the_task_manager.generate_a_ball()
            elif state == DemoStates.SCAN:
                if the_task.state not in [TaskStates.WORKING]:
                    rospy.loginfo(f'=== PickNDrop SCAN: Scan Aborted and State changed to STOP')
                    state = DemoStates.STOP                    
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
                    rospy.loginfo(f'=== PickNDrop PICK: Pick Failed and State changed to STOP')
                    state = DemoStates.STOP
                else:
                    rospy.loginfo(f'=== PickNDrop PICK: Pick Successful and Submit Drop Task')
                    self.the_blackboard.attached_object = True
                    task_manager.submit_task(the_task := DropObjectTask()) 
                    state = DemoStates.DROP
            elif state == DemoStates.DROP:
                the_task.wait_for_completion() 
                if the_task.get_state() in [TaskStates.SUCCEEDED]:
                    rospy.loginfo(f'=== PickNDrop DROP: Drop Successful')
                    self.the_blackboard.attached_object = False
                    state = DemoStates.INIT
                else:
                    rospy.loginfo(f'=== PickNDrop PICK: Drop Failed and State changed to STOP')
                    state = DemoStates.STOP
            elif state == DemoStates.STOP:
                rospy.loginfo(f'=== PickNDrop STOP')   
                if the_task is not None and the_task.get_state() == TaskStates.GUARD_ABORTED:
                    rospy.logwarn(f'Aborted by the EStop -> reset the task manager') 
                time.sleep(10.0)
                rospy.loginfo(f'Attempt to recover -> release the E-Stop button')       
                while True:
                    if self.estop_status == False:
                        break
                    time.sleep(1.0)
                rospy.loginfo(f'Reset the guard of the task manager') 
                self.the_task_manager.reset_guard()
                if self.the_blackboard.attached_object:
                    task_manager.submit_task(the_task := DropObjectTask())                    
                    state = DemoStates.DROP
                else:                   
                    state = DemoStates.INIT                    

if __name__=='__main__':
    rospy.init_node('run_pnd_task_manager_demo', anonymous=False)
    try:
        GuardedPNDDemoApplication()
        rospy.loginfo('pnd_task_manager_demo is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


