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

# general modules
import signal, sys, threading, time
import rospy, actionlib
from arm_commander.commander_moveit import GeneralCommander, logger
from task_trees.msg import TaskAction, TaskGoal, TaskResult
from task_trees.states import COMPLETION_STATES
from demos.pushblock.task_trees_manager_pushblock import *

class PushBlockROSServer():
    """ The server side of the application program for the PushBlock Demo that implements the behaviour tree
        and an action server for receiving Goals from the client application.
    """
    def __init__(self):
        # - create the stop signal handler
        signal.signal(signal.SIGINT, self.stop)
        rospy.on_shutdown(self.cb_shutdown)
        # set constant
        self.TIMEOUT = 30.0 # seconds
        # synchronization lock
        self.task_lock = threading.Lock()
        # create action server
        self.the_action_server = actionlib.SimpleActionServer('/pushblock/go', 
                            TaskAction, execute_cb=self.received_goal, auto_start=False)
        self.the_action_server.register_preempt_callback(self.received_preemption)
        self.the_action_server.start() 
        # setup the task trees manager
        self.arm_commander = GeneralCommander('panda_arm')
        self.the_task_manager = PushBlockTaskTreesManager(self.arm_commander)
        
        self.the_task_manager.setup_objects() 
        self.the_task_manager.generate_the_object()
        
    # -- callback function for shutdown
    def cb_shutdown(self):
        logger.info('the ros node is being shutdown')
        sys.exit(0)

    def stop(self, *args, **kwargs):
        logger.info('the ros node is being stopped')
        sys.exit(0)

    # -- callback for receiving the goal of action
    def received_goal(self, goal):
        logger.info(f'goal received: {goal.target}')
        result = TaskResult()
        self.task_lock.acquire()
        try:
            target_area = goal.target
            if target_area >= 1 and target_area <= 4:
                self.the_task_manager.submit_task(task:=PushBlockTask(f'area_{target_area}'))
            elif target_area == 0:
                self.the_task_manager.submit_task(task:=MoveNamedPoseTask(f'named_poses.home'))
            else:
                result.data = 'ABORTED DUE TO INVALID TARGET'
                self.the_action_server.set_aborted(result) 
                return
            self.task = task
            t = time.time()
            while self.task is not None:
                time.sleep(0.1)
                if self.task.get_state() in COMPLETION_STATES:
                    break
                if time.time() - t > self.TIMEOUT:
                    logger.error(f"ros_server: the task is cancelled due to timeout ({self.TIMEOUT} s)")
                    result.data = 'TIMEOUT'
                    self.task.cancel()
                    self.the_action_server.set_aborted(result)
                    return
                if self.the_action_server.is_preempt_requested():
                    break
            # finalize the result
            if self.the_action_server.is_preempt_requested():
                result.data = 'ABORTED_DUE_TO_CANCEL_GOAL'
                self.the_action_server.set_aborted(result)
            elif self.task.state in [TaskStates.ABORTED, TaskStates.FAILED]:
                result.data = self.task.result
                self.the_action_server.set_aborted(result) 
            elif self.task.state in [TaskStates.INVALID]:
                result.data = 'INVALID_TASK_DUE_TO_SYSTEM_ERROR'
                self.the_action_server.set_aborted(result)                        
            else:
                self.the_action_server.set_succeeded(result)
        finally:
            self.task_lock.release()

    # -- callback for receiving preemption of the goal
    def received_preemption(self):
        self.task_lock.acquire()
        try:
            logger.info(f'preemption received')
            if self.task is not None:
                self.task.cancel()
        finally:
            self.task_lock.release()

# ----- the main program
if __name__ == '__main__':
    """ A script for starting an action server that accept action goal for the pushblock demo
    """
    rospy.init_node('pushblock_server', anonymous=False)
    try:
        the_action_server = PushBlockROSServer()
        logger.info('Pushblock ROS server (pushblock_server) is running')
    except Exception as e:
        logger.exception(e)