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

import os, random, sys, signal, threading, time
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.trees import BehaviourTree

# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
from task_trees.behaviours_move import DoMoveNamedPose, DoMoveXYZ, DoRotate
from task_trees.task_scene import Scene

# The TaskManager specialized for this application
class SimpleMovePyTreesApplication():
    """ This is an application building a behaviour tree using py-trees and the extension behaviour set from the task tree SDK.
        It illustrates how to integrate with the task scene and the use of scene configuration file
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
        self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene.yaml'))
        # setup name poses
        self.named_poses = self.the_scene.key_list_under_config_key('named_poses')
        for pose_name in self.named_poses:
            pose_name = 'named_poses.' + pose_name
            self.arm_commander.add_named_pose(pose_name, self.the_scene.query_config(pose_name))

        # build the behaviour tree
        self.root_sequence = self.create_move_branch()
        self.bt = BehaviourTree(self.root_sequence) 
        # py_trees.display.render_dot_tree(self.bt.root)
        
        # spin the tree
        self.the_thread = threading.Thread(target=lambda: self.bt.tick_tock(period_ms=spin_period_ms), daemon=True)
        self.the_thread.start()  

    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    # A behaviour tree branch that moves to a specified named poses
    def create_move_branch(self) -> Composite:
        """ Return a behaviour tree branch that moves to two named poses defined in the task scene config file
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = Sequence (
                'move_branch',
                memory=True,
                children=[
                    DoMoveNamedPose('move_to_stow', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.stow'),
                    DoMoveNamedPose('move_to_home', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),                     
                    ],
        )
        return move_branch

    # The callback for the signal resulting from SIGINT pressing CTRL-C 
    def stop(self, *args, **kwargs):
        sys.exit(0)

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
      


