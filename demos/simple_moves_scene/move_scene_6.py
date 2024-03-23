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

import os, random
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector
# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
from task_trees.behaviours_move import DoMoveMultiXYZ
from task_trees.task_trees_manager import TaskTreesManager
from task_trees.task_scene import Scene

# ---------------------------------------
# The TaskManager specialized for this application
class SceneMoveTaskManager(TaskTreesManager):
    """ This is a subclass of TaskManager, for illustration of using the framework for developing behaviour trees
        The behaviour tree contains one behaviour, which moves to multi xyz poses, each of which is a compositional value of logical or late binding.
        Specifically, the multi poses form a virtual rectangle with x values (near, far), y values (start, end), and z value a random change from
        the previous z value
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
        
        self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene_6.yaml'))
        # build and install the behavior tree
        self._add_priority_branch(self.create_move_branch())
        
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
    
    # --------------------------------------------------
    # --- functions for behaviour trees to generate late binding target poses
    def generate_random_z_change(self) -> list:
        xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy()
        xyz = [None, None, xyzrpy[2] + random.uniform(-0.1, 0.1)]
        logger.info(f'generate_random_xyz: {xyz}')
        return xyz

    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    # A behaviour tree branch that moves to a specified position xyz
    def create_move_branch(self) -> Composite:
        """ Return a behaviour tree branch that moves to multi xyz poses, each of which is a compositional value of logical or late binding
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = py_trees.composites.Sequence(
                'move_branch',
                memory=True,
                children=[
                    DoMoveMultiXYZ('move_multi_xyz', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz_list=[
                                        ('positions.start', 'positions.near', self.generate_random_z_change),
                                        ('positions.end', 'positions.near', self.generate_random_z_change),
                                        ('positions.end', 'positions.far', self.generate_random_z_change),
                                        ('positions.start', 'positions.far', self.generate_random_z_change),                                     
                                     ]), 
                    ],
        )
        return move_branch
    
if __name__=='__main__':
    # rospy.init_node('simple_move_example', anonymous=False)
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_task_manager = SceneMoveTaskManager(arm_commander)
        # display the behaviour tree as an image
        # the_task_manager.display_tree(target_directory=os.path.dirname(__file__))
        logger.info('simple_move_example is running')
        the_task_manager.spin()
    except Exception as e:
        logger.exception(e)
      


