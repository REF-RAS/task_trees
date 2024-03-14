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
from task_trees.behaviours_move import DoMovePose
from task_trees.task_trees_manager import TaskTreesManager

# ---------------------------------------
# The TaskManager specialized for this application
class SimpleMoveTaskManager(TaskTreesManager):
    """ This is a subclass of TaskManager, for illustration of using the framework for developing behaviour trees
        The behaviour tree contains one behaviour, which moves to a new pose generated from the current one with one component changed by a random number
        The end-effector continuously move between these constrained random poses
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(SimpleMoveTaskManager, self).__init__(arm_commander)

        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()

        # build and install the behavior tree
        self._add_priority_branch(self.create_move_branch())
        
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
        
    # --------------------------------------------------
    # --- functions for behaviour trees to generate late binding target poses    
    def generate_random_move(self) -> list:
        xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy()
        which_dim = random.randint(0, 4)
        if which_dim in [0, 1, 2]:
            xyzrpy[which_dim] += random.uniform(-0.2, 0.2)  # the x, y, or z component
        elif which_dim == 3:
            xyzrpy[5] += random.uniform(-1.57, 1.57)        # the yaw component
        rospy.loginfo(f'generate_random_move: {xyzrpy}')
        return xyzrpy

    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    # returns a behaviour tree branch that move the end_effector to random positions
    def create_move_branch(self) -> Composite:
        """ Returns a behaviour tree branch that move the end_effector to shift positions in a random step in one component
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = py_trees.composites.Sequence(
                'move_branch',
                memory=True,
                children=[
                    DoMovePose('move_random_step', True, arm_commander=self.arm_commander, target_pose=self.generate_random_move), 
                    ],
        )
        return move_branch
    
if __name__=='__main__':
    rospy.init_node('simple_move_example', anonymous=False)
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_task_manager = SimpleMoveTaskManager(arm_commander)
        # display the behaviour tree as an image
        # the_task_manager.display_tree(target_directory=os.path.dirname(__file__))
    
        rospy.loginfo('simple_move_example is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
      


