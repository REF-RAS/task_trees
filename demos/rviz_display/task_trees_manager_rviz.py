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

import os, math, random
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector

# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger

from task_trees.behaviours_move import DoMoveNamedPose, DoMoveXYZ, DoMoveXYZRPY, DoRotate
from task_trees.task_trees_manager import TaskTreesManager, BasicTask
from task_trees.task_scene import Scene
from task_trees.scene_to_rviz import SceneToRViz
# -------------------------------------
# Tasks specialized for this demo

# A sequence of behaviours for moving the arm to one of the named pose 
class MoveNamedPoseTask(BasicTask):
    def __init__(self, named_pose:str):
        """ the constructor
        :param named_pose: one of the named pose defined in the configuration file
        :type named_pose: str
        """
        if named_pose is None or type(named_pose) is not str or len(named_pose) == 0:
            logger.error(f'{__class__.__name__}: parameter (named_pose) is not an non-empty string -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter is invalid')  
        super(MoveNamedPoseTask, self).__init__(named_pose)  

# ----------------------------------------
# The TaskManager specialized for this application
class RVizTaskTreesManager(TaskTreesManager):
    """ This is a subclass specialized for the rviz demo
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(RVizTaskTreesManager, self).__init__(arm_commander)
        # load the task scene
        config_file = os.path.join(os.path.dirname(__file__), 'task_scene.yaml')
        self.the_scene = Scene(config_file)
        # setup visualization in rviz
        self.scene_to_rviz = SceneToRViz(self.the_scene, arm_commander.get_world_reference_frame(), False)
        self.scene_to_rviz.display_object('objects.teapot')

        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        bird_object = self.the_scene.get_object_config('teapot')
        self.arm_commander.add_object_to_scene('teapot_another', bird_object.model_file, bird_object.dimensions, 
                                bird_object.xyz, bird_object.rpy, reference_frame='area')
        # self.arm_commander.set_workspace_walls(*(self.the_scene.query_config('regions.workspace')))
        # setup name poses
        self._define_named_poses(self.the_scene)
        self._define_custom_frames(self.the_scene)
            
        # setup the blackboard and the two blackboard keys 'seen_object'
        self.the_blackboard.register_key(key='the_object', access=py_trees.common.Access.WRITE)      
       
        # build and install the behavior tree
        self._set_initialize_branch(self.create_initialize_branch())
        self._add_task_branch(self.create_move_namedpose_task_branch(), MoveNamedPoseTask)
        
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
 
    # return the logical pose of the accepted task, or raise TypeError if no task is submitted
    def query_logical_pose_of_task(self):
        """ return the goal, which is a logical pose, of the current task
        :raises TypeError: no task has been submitted
        :return: the logical pose
        :rtype: unspecified as defined by the specific subclass of BasicTask
        """
        if self.the_blackboard.exists('task'):
            return self.the_blackboard.task.get_param()
        raise TypeError(f'unable to query logical pose due to no task has been submitted')

    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    def create_move_namedpose_task_branch(self) -> Composite:
        # - the branch that executes the task MoveNamedPoseTask
        move_named_pose_branch = py_trees.decorators.Timeout(
            duration=60,
            name='move_named_pose_branch_timeout',
            child=py_trees.composites.Sequence(
                'move_named_pose_branch',
                memory=True,
                children=[
                    DoMoveNamedPose('move_home_first', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),
                    DoMoveNamedPose('move_task_pose', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose=self.query_logical_pose_of_task),                    
                    ],
            ),
        )
        return move_named_pose_branch
    
    def create_initialize_branch(self) -> py_trees.decorators.OneShot:
        init_action_branch = py_trees.composites.Sequence(
            'init_action_branch',
            memory=True,
            children = [
                DoMoveNamedPose('init_move_home', True,
                                        arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),               
            ]
        )
        return init_action_branch

      


