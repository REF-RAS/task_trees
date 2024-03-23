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

import threading, time, signal, sys, random
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.decorators import OneShot
from py_trees.trees import BehaviourTree

# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger

from task_trees.behaviours_move import DoMoveXYZRPY

# The TaskManager specialized for this application
class SimpleMovePyTreesApplication():
    """ This is an application building a behaviour tree using py-trees and the extension behaviour set from the task tree SDK
        The behaviour tree contains one behaviour, which moves to a RANDOM pose specified in xyz and rpy.
        The end-effector continuously move between random positions and rotations.  Some poses are invalid (due to self-collision).
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        signal.signal(signal.SIGINT, self.stop)
        # setup the robotic manipulation platform through the commander
        self.arm_commander:GeneralCommander = arm_commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        self.arm_commander.wait_for_ready_to_move()
        # build the behaviour tree
        self.move_branch = self.create_move_branch()
        self.init_branch = self.create_init_branch()
        self.init_branch = OneShot('init_oneshot_branch', policy = py_trees.common.OneShotPolicy.ON_COMPLETION, 
                                    child=self.init_branch)
    
        self.root_sequence = Sequence('root_sequence', memory=True, children=[
            self.init_branch,
            self.move_branch,
            ])
        self.bt = BehaviourTree(self.root_sequence) 
        # py_trees.display.render_dot_tree(self.bt.root)
        
        # spin the tree
        self.the_thread = threading.Thread(target=lambda: self.bt.tick_tock(period_ms=spin_period_ms), daemon=True)
        self.the_thread.start()  
    
    # --------------------------------------------------
    # --- functions for behaviour trees to generate late binding target poses
    def generate_random_xyz(self) -> list:
        xyz = [random.uniform(0.1, 0.5), random.uniform(-0.3, 0.3), random.uniform(0.2, 0.6)]
        logger.info(f'generate_random_xyz: {xyz}')
        return xyz
    
    def generate_random_rpy(self) -> list:
        logger.info(f'{self.arm_commander.pose_in_frame_as_xyzrpy()}')
        rpy = [random.uniform(3.14, 3.14), random.uniform(0, 0), random.uniform(-3.14, 3.14)]
        logger.info(f'generate_random_xyz: {rpy}')
        return rpy
    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    # returns a behaviour tree branch that move the end_effector to random positions and rotations
    def create_move_branch(self) -> Composite:
        """ Returns a behaviour tree branch that move the end_effector to random positions and rotations
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = Sequence (
                'move_branch',
                memory=True,
                children=[
                    DoMoveXYZRPY('move_xyzrpy', True, arm_commander=self.arm_commander, 
                                 target_xyz=[[0.3, None, None], self.generate_random_xyz],
                                 target_rpy=self.generate_random_rpy), 
                    ],
        )
        return move_branch

    # The callback for the signal resulting from SIGINT pressing CTRL-C 
    def stop(self, *args, **kwargs):
        sys.exit(0)

    # returns a behaviour tree branch that performs initialzation of the robot by moving to a prescribed pose
    def create_init_branch(self) -> Composite:
        # - the branch that executes the task MoveNamedPoseTask
        init_branch = py_trees.composites.Sequence(
                'init_branch',
                memory=True,
                children=[
                    DoMoveXYZRPY('reset_pose', True, arm_commander=self.arm_commander, target_xyz=[0.307, 0.0, 0.588],
                                 target_rpy=[3.139, 0.0, -0.785]), 
                    ],
        )
        return init_branch  
    
if __name__=='__main__':
    # rospy.init_node('simple_move_example', anonymous=False)
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_application = SimpleMovePyTreesApplication(arm_commander)
        logger.info('The simple_move_example is running')
        # prevent the main thread from exit the program
        while True: time.sleep(1.0)
    except Exception as e:
        logger.exception(e)
      


