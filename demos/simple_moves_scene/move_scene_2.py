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

from time import sleep
import operator, yaml, os, math, random, copy, sys, signal, threading, random
from math import isclose
import rospy
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.trees import BehaviourTree

# robot control module
from arm_commander.commander_moveit import GeneralCommander
import arm_commander.moveit_tools as moveit_tools

from task_trees.states import TaskStates
from task_trees.behaviours_base import *
from task_trees.behaviours_move import DoMoveNamedPose, DoMoveXYZ, DoRotate
from task_trees.task_trees_manager import TaskTreesManager
from task_trees.task_scene import Scene

# ---------------------------------------
# The TaskManager specialized for this application
class SceneMoveTaskManager(TaskTreesManager):
    """ This is a subclass of TaskManager, for illustration of using the framework for developing behaviour trees
        The behaviour tree contains one behaviour, which moves to a pose specified in xyz
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(SceneMoveTaskManager, self).__init__(arm_commander)

        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        
        self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene_2.yaml'))
        # build and install the behavior tree
        self._add_priority_branch(self.create_move_branch())
        
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
    
    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    # A behaviour tree branch that moves to a specified position xyz
    def create_move_branch(self) -> Composite:
        """ Return a behaviour tree branch that moves between two specified positions in xyz
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = py_trees.composites.Sequence(
                'move_branch',
                memory=True,
                children=[
                    DoMoveXYZ('move_start', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='positions.start'), 
                    DoMoveXYZ('move_end', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='positions.end'), 
                    ],
        )
        return move_branch
    
if __name__=='__main__':
    rospy.init_node('simple_move_example', anonymous=False)
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_task_manager = SceneMoveTaskManager(arm_commander)
        # display the behaviour tree as an image
        # the_task_manager.display_tree(target_directory=os.path.dirname(__file__))
    
        rospy.loginfo('simple_move_example is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
      


