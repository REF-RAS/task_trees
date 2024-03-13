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
import rospy, actionlib
from actionlib_msgs.msg import GoalStatus
from task_trees.msg import TaskAction, TaskGoal, TaskFeedback, TaskResult

# -- Test cases specialized for the Push Block Demo
class PushBlockROSClientDemo():
    """ The application program for the PushBlock Demo implemented as a ROS client communicating with behaviour tree
        on the server side 
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        
        self.action_client = actionlib.SimpleActionClient('/pushblock/go', TaskAction)
        rospy.loginfo(f'Waiting for ROS action server at /pushblock/go')
        self.action_client.wait_for_server()
        rospy.loginfo(f'The ROS action server is found')
        self._run_demo()

    def stop(self, *args, **kwargs):
        rospy.loginfo('stop signal received')
        sys.exit(0)
       
    def _run_demo(self):
        rospy.loginfo(f'=== Submit a ROS Action Goal for MoveNamedPoseTask (home)')
        goal = TaskGoal(target=0)
        status = self.action_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(30.0), 
                                            preempt_timeout=rospy.Duration(30.0))  
        if status == GoalStatus.SUCCEEDED:
            print(f'Moved to the home pose: SUCCEEDED')
        elif status == GoalStatus.ABORTED:
            print(f'Failed to move to the home pose: the error is {self.action_client.get_result()}') 
        # initialize variables
        current_area = 1
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
                    rospy.loginfo('Starting to generate random targets')
                if target == 'H':
                    goal = TaskGoal(target=0)
                    status = self.action_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(30.0), 
                                            preempt_timeout=rospy.Duration(30.0)) 
                    continue
            if is_random:
                target = random.randint(1, 4)
            if current_area == target:
                if not is_random:
                    rospy.loginfo(f'The area "{target}" is where the block is already located')
                continue
            rospy.loginfo(f'=== Submit a ROS Action Goal PushBlockTask ({target})')
            goal = TaskGoal(target=int(target))
            status = self.action_client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(30.0), 
                                            preempt_timeout=rospy.Duration(30.0)) 
            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo(f'The Action Goal SUCCEEDED')
                current_area = target
            else:
                rospy.logwarn(f'The Action Goal ABORTED or FAILED')

if __name__=='__main__':
    rospy.init_node('run_pnd_ros_demo', anonymous=False)
    try:
        PushBlockROSClientDemo()
        rospy.loginfo('run_pnd_ros_demo is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


