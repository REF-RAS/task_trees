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
from collections import namedtuple, defaultdict
import rospy
from std_msgs.msg import Float32
from py_trees.common import Status
# robot control module
from task_trees.behaviours_base import ConditionalBehaviour
from task_scene_gridscan import GridScanScene
from task_trees.tools import logger
from demos.gridscan.behaviours_gridscan import DoMoveTankGrid

# -----------------------------------------------------------------------------------------------------
# Advanced behaviours for illustrating the versatility of the ConditionalCommanderBehaviour base class

# This class demonstrates the Behaviour Exception pattern
class DoMoveTankGridVisualCDROS(DoMoveTankGrid):
    """ This behaviour is based on DoMoveXYAtTankFixedZOrientation that moves the end-effector to a pose defined in the frame of the tank in a cartesian manner. 
        The roll and pitch components of the orientation of the end-effector is fixed.
        In addition, it uses the readings from a range finder (or other types of sensors) and cancel the command if collision is imminent.
        This behaviour is coupled to ROS and saves the readings from a ROS topic to an instance variable
    """   
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene:GridScanScene=None, grid_position=None, constraint_fn=None, reference_frame=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param logical_pose: the tank logical pose, which is defined (tile_x, tile_y, cell_x, cell_y), or an alias as a string defined in the task scene
        :type logical_pose: a list of 4 integers, a string, or a function that returns the planar logical pose
        :param z_level_pose: the z_level pose
        :type z_level_pose: a string representing the z_level_pose as defined in the task scene
        """        
        super(DoMoveTankGridVisualCDROS, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, arm_commander=arm_commander, scene=scene, 
                                                        grid_position=grid_position, constraint_fn=constraint_fn, reference_frame=reference_frame)
        self.MIN_DISTANCE = 10.0
        # subscribe to the ROS topic that publishes the readings of a range finder
        self.range_finder_sub = rospy.Subscriber('/sensor/range_finder', Float32, self._cb_range_finder_update)
        self.current_dist = None
        self.previous_too_close = None # a variable for recording the previous state of too_close to reduce the verbosity of the info output
    
    # the callback for the ROS topic subscription
    def _cb_range_finder_update(self, msg:Float32):
        self.current_dist = msg.data
        
    # the concrete implementation of the logic when the General Commander is READY     
    def update_when_busy(self):
        # the logic for detecting imminant collision
        too_close = False if self.current_dist is None else self.current_dist < self.MIN_DISTANCE
        # less verbose output of the distance check
        if self.previous_too_close is None or self.previous_too_close != too_close:
            logger.info(f'DoMoveXYAtTankVisualCDROS: too_close: {too_close}')
            self.previous_too_close = too_close
        # ask the General Commander to cancel the command and return FAILURE if too close is True 
        if too_close:
            logger.error(f'DoMoveXYAtTankVisualCDROS ({self.name}): visual collision detection alert (too close)')
            self.arm_commander.abort_move()
            the_task = self.the_blackboard.task
            the_task.result = f'INTERRUPTED RAISED BY VISUAL COLLISION DETECTION (distance too close: {self.current_dist})'
            return Status.FAILURE   
        return Status.RUNNING

# This class demonstrates the Behaviour Exception pattern
class DoMoveTankGridVisualCDBlackboard(DoMoveTankGrid):
    """ This behaviour is based on DoMoveXYAtTankFixedZOrientation that moves the end-effector to a pose defined in the frame of the tank in a cartesian manner. 
        The roll and pitch components of the orientation of the end-effector is fixed.
        In addition, it uses the readings from a range finder (or other types of sensors) and cancel the command if collision is imminent.
        This behaviour is not ROS based and assumes the readings is in the blackboard
    """   
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene:GridScanScene=None, grid_position=None, constraint_fn=None, reference_frame=None):        
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param logical_pose: the tank logical pose, which is defined (tile_x, tile_y, cell_x, cell_y), or an alias as a string defined in the task scene
        :type logical_pose: a list of 4 integers, a string, or a function that returns the planar logical pose
        :param z_level_pose: the z_level pose
        :type z_level_pose: a string representing the z_level_pose as defined in the task scene
        """  
        super(DoMoveTankGridVisualCDBlackboard, self).__init__(name=name, condition_fn=condition_fn, constraint_policy=condition_policy, 
                                                               arm_commander=arm_commander, scene=scene, grid_position=grid_position, 
                                                               constraint_fn=constraint_fn, reference_frame=reference_frame)
        self.MIN_DISTANCE = 10.0
    # the concrete implementation of the logic when the General Commander is READY     
    def update_when_busy(self):
        if not self.the_blackboard.exists('range_finder'):
            return Status.RUNNING
        current_dist = self.the_blackboard.range_finder
        too_close = False if current_dist is None else current_dist< self.MIN_DISTANCE
        if self.previous_too_close is None or self.previous_too_close != too_close:
            logger.info(f'DoMoveXYAtTankVisualCDROS: too_close: {too_close}')
            self.previous_too_close = too_close
        # ask the General Commander to cancel the command and return FAILURE if too close is True 
        if too_close:
            logger.error(f'DoMoveXYAtTankVisualCDROS ({self.name}): visual collision detection alert (too close)')
            self.arm_commander.abort_move()
            the_task = self.the_blackboard.task
            the_task.result = f'INTERRUPTED RAISED BY VISUAL COLLISION DETECTION (distance too close: {current_dist})'
            return Status.FAILURE   
        return Status.RUNNING
    

