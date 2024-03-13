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
from task_trees.behaviours_move import DoMoveXYZRPY, DoMoveDisplaceXYZ, DoMoveXYZ
from task_trees.task_trees_manager import TaskTreesManager, BasicTask

class MoveRectTask(BasicTask):
    def __init__(self):
        """ the constructor for a task moving in a rectangle pattern
        """
        super(MoveRectTask, self).__init__()

class MoveRandomTask(BasicTask):
    def __init__(self, min_y:float, max_y:float):
        """ the constructor for a task moving to three random locations along the y axis
        """
        super(MoveRandomTask, self).__init__()
        self.min_y = min_y
        self.max_y = max_y

# ---------------------------------------
# The TaskManager specialized for this application
class SimpleTaskMoveManager(TaskTreesManager):
    """ This is a subclass of TaskManager, for illustration of using the framework for developing behaviour trees
        The behaviour tree contains one behaviour, which moves the end effector in a rectangular circuit, each side divided into 5 steps of displacement
        On the last side of the rectangle, the step size is random between 0.01 and 0.09 meters. 
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(SimpleTaskMoveManager, self).__init__(arm_commander)
        # build and install the behavior tree
        self._set_initialize_branch(self.create_init_branch())
        self._add_task_branch(self.create_move_rect_branch(), MoveRectTask)
        self._add_task_branch(self.create_move_3_random_branch(), MoveRandomTask)        

        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
    
    # --------------------------------------------------
    # --- functions for behaviour trees to generate late binding target poses
    def generate_random_dxyz(self) -> list:
        dxyz = [0.0, 0.0, random.uniform(-0.25, -0.35)]
        rospy.loginfo(f'generate_random_dxyz: {dxyz}')
        return dxyz
    
    
    def generate_random_xyz(self) -> list:
        if not self.the_blackboard.exists('task'):
            raise TypeError(f'unable to generate random pose due to no task has been submitted')
        min_y = self.the_blackboard.task.min_y
        max_y = self.the_blackboard.task.max_y           
        xyz = [None, random.uniform(min_y, max_y), None]
        rospy.loginfo(f'generate_random_xyz: {xyz}')
        return xyz
    
    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    # returns a behaviour tree branch that move the end_effector in a rectanglar
    def create_move_rect_branch(self) -> Composite:
        """ Returns a behaviour tree branch that move the end_effector in a rectangular path
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = py_trees.composites.Sequence(
                'move_branch',
                memory=True,
                children=[
                    DoMoveDisplaceXYZ('move_dy', True, arm_commander=self.arm_commander, dxyz=[0.0, 0.3, 0]), 
                    DoMoveDisplaceXYZ('move_dz', True, arm_commander=self.arm_commander, dxyz=[0, 0, 0.3]), 
                    DoMoveDisplaceXYZ('move_ndy', True, arm_commander=self.arm_commander, dxyz=[0, -0.3, 0]), 
                    DoMoveDisplaceXYZ('move_random_ndz', True, arm_commander=self.arm_commander, dxyz=self.generate_random_dxyz),               
                    ],
        )
        return move_branch
    
    # returns a behaviour tree branch that move the end_effector to random positions and rotations
    def create_move_3_random_branch(self) -> Composite:
        """ Returns a behaviour tree branch that move the end_effector in a rectangular path
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = py_trees.composites.Sequence(
                'move_branch',
                memory=True,
                children=[
                        DoMoveXYZ('move_random_y_1', True, arm_commander=self.arm_commander, target_xyz=self.generate_random_xyz), 
                        DoMoveXYZ('move_random_y_2', True, arm_commander=self.arm_commander, target_xyz=self.generate_random_xyz), 
                        DoMoveXYZ('move_random_y_3', True, arm_commander=self.arm_commander, target_xyz=self.generate_random_xyz),                                       
                    ],
        )
        return move_branch   
    
    # returns a behaviour tree branch that performs initialzation of the robot by moving to a prescribed pose
    def create_init_branch(self) -> Composite:
        # - the branch that executes the task MoveNamedPoseTask
        init_branch = py_trees.composites.Sequence(
                'init_branch',
                memory=True,
                children=[
                    DoMoveXYZRPY('reset_pose', True, arm_commander=self.arm_commander, target_xyz=[0.3, -0.2, 0.3],
                                 target_rpy=[3.139, 0.0, -0.785]), 
                    ],
        )
        return init_branch        
   
   
class TaskDemoApplication():
    """ The application program for the simple Task Demo
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='the_object', access=py_trees.common.Access.READ)  
        
        self.arm_commander = GeneralCommander('panda_arm')
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        self.arm_commander.wait_for_ready_to_move()
        self.the_task_manager = SimpleTaskMoveManager(self.arm_commander)
        # self.the_task_manager.display_tree()
        self._run_demo()
        rospy.spin()
        
    def stop(self, *args, **kwargs):
        rospy.loginfo('stop signal received')
        self.the_task_manager.shutdown()
        sys.exit(0)
        
    def _run_demo(self):
        task_manager = self.the_task_manager
        the_task = None
        rospy.sleep(5.0)
        rospy.loginfo(f'=== Task Demo Started') 
        while True:
            while True:
                target = input('Submit task MoveRectTask (1), MoveRandomTask (2) or Quit (Q): ')
                if target in ['1', '2', 'Q']:
                    break
                rospy.sleep(0.1)
            if target == 'Q':
                sys.exit(0)
            elif target == '1':
                task_manager.submit_task(the_task:=MoveRectTask())
                the_task.wait_for_completion() 
            elif target == '2':
                task_manager.submit_task(the_task:=MoveRandomTask(-0.3, 0.3))
                the_task.wait_for_completion() 

if __name__=='__main__':
    rospy.init_node('simple_task_demo', anonymous=False)
    try:
        TaskDemoApplication()
        rospy.loginfo('simple_task_demo is running')
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
      


