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

from task_trees.behaviours_move import DoMovePose

# The TaskManager specialized for this application
class SimpleMovePyTreesApplication():
    """ This is an application building a behaviour tree using py-trees and the extension behaviour set from the task tree SDK
        The behaviour tree contains two behaviours, each of which moves to a pose specified in xyzrpy
        The end-effector is moving between the two poses
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """

        # setup the robotic manipulation platform through the commander
        self.arm_commander:GeneralCommander = arm_commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        self.arm_commander.wait_for_ready_to_move()

        # build the behaviour tree
        self.root_sequence = self.create_move_branch()
        self.bt = BehaviourTree(self.root_sequence) 
        # py_trees.display.render_dot_tree(self.bt.root)
        
        # spin the tree
        self.the_thread = threading.Thread(target=lambda: self.bt.tick_tock(period_ms=spin_period_ms), daemon=True)
        self.the_thread.start()  
    
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
                    DoMovePose('move_xyzrpy', True, arm_commander=self.arm_commander, target_pose=[0.5, 0.0, 0.2, 3.14, 0, 1.58]), 
                    DoMovePose('move_xyzrpy', True, arm_commander=self.arm_commander, target_pose=[0.5, 0.0, 0.5, 3.14, 0.2, 0]), 
                    ],
        )
        return move_branch
    
if __name__=='__main__':
    rospy.init_node('simple_move_example', anonymous=False)
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_task_manager = SimpleMovePyTreesApplication(arm_commander)
    
        rospy.loginfo('simple_move_example is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
      


