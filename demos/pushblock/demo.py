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
from task_trees_manager_pushblock import *

# -- Test cases specialized for the Push Block Demo
class PushBlockDemoApplication():
    """ The application program for the PushBlock Demo
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='the_object', access=py_trees.common.Access.READ)  
        
        self.arm_commander = GeneralCommander('panda_arm')
        self.the_task_manager = PushBlockTaskTreesManager(self.arm_commander)
        # self.the_task_manager.display_tree()
        
        self._run_demo()
        rospy.spin()

    def stop(self, *args, **kwargs):
        rospy.loginfo('stop signal received')
        self.the_task_manager.shutdown()
        sys.exit(0)

    def _run_test(self):
        task_manager = self.the_task_manager
        rospy.loginfo(f'=== Test 1')         
        rospy.loginfo(f'=== Submit a MoveNamedPoseTask (stow)')
        task_manager.submit_task(the_task:=MoveNamedPoseTask('named_poses.stow'))
        the_task.wait_for_completion()
        rospy.loginfo(f'=== Submit a MoveNamedPoseTask (home)')
        task_manager.submit_task(the_task:=MoveNamedPoseTask('named_poses.home'))
        the_task.wait_for_completion()   
        rospy.loginfo(f'=== Test Finished')
             
    def _run_push_test(self): 
        task_manager = self.the_task_manager
        the_task = None
        rospy.loginfo(f'=== PushBlock Demo Started') 
        rospy.loginfo(f'=== Submit a MoveNamedPoseTask (home)')
        task_manager.submit_task(the_task:=MoveNamedPoseTask('named_poses.home'))
        the_task.wait_for_completion()    
        # generate the block for simulation          
        task_manager.generate_the_object()
        # test push blocks to different areas
        rospy.loginfo(f'=== Submit a PushBlockTask (2)')
        task_manager.submit_task(the_task:=PushBlockTask('area_2'))
        the_task.wait_for_completion()          
        rospy.loginfo(f'=== Submit a PushBlockTask (4)')
        task_manager.submit_task(the_task:=PushBlockTask('area_4'))
        the_task.wait_for_completion()   
        rospy.loginfo(f'=== Submit a PushBlockTask (3)')
        task_manager.submit_task(the_task:=PushBlockTask('area_3'))
        the_task.wait_for_completion()            
        rospy.loginfo(f'=== Submit a PushBlockTask (1)')
        task_manager.submit_task(the_task:=PushBlockTask('area_1'))
        the_task.wait_for_completion()          

    def _run_demo(self):
        task_manager = self.the_task_manager
        the_task = None
        rospy.loginfo(f'=== PushBlock Demo Started') 
        rospy.loginfo(f'=== Submit a MoveNamedPoseTask (home)')
        task_manager.submit_task(the_task:=MoveNamedPoseTask('named_poses.home'))
        the_task.wait_for_completion()   
        # generate the block for simulation 
        task_manager.setup_objects()         
        task_manager.generate_the_object()
        # initialize variables
        is_random = False
        while True:
            if not is_random:
                while True:
                    target = input('Enter the area (1, 2, 3, 4), go home (H), or run random demo (R): ')
                    if target in ['1', '2', '3', '4', 'H', 'R']:
                        break
                    rospy.sleep(0.1)
                if target == 'R':
                    is_random = True
                if target == 'H':
                    task_manager.submit_task(the_task:=MoveNamedPoseTask('named_poses.home'))
                    the_task.wait_for_completion() 
                    continue
            if is_random:
                target = random.randint(1, 4)
            area = f'area_{target}'
            if self.the_blackboard.the_object['area'] == area:
                if not is_random:
                    rospy.loginfo(f'The area "{target}" is where the block is already located')
                continue
            rospy.loginfo(f'=== Submit a PushBlockTask ({area})')
            task_manager.submit_task(the_task:=PushBlockTask(area))
            the_task.wait_for_completion()

if __name__=='__main__':
    rospy.init_node('run_pnd_task_manager_demo', anonymous=False)
    try:
        PushBlockDemoApplication()
        rospy.loginfo('pnd_task_manager_demo is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


