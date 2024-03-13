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
import operator, yaml, os, math, random, copy, sys, signal, threading
from math import isclose
import rospy
import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.trees import BehaviourTree
from moveit_msgs.msg import Constraints

# robot control module
from arm_commander.commander_moveit import GeneralCommander
import arm_commander.moveit_tools as moveit_tools
from task_scene_gridscan import GridScanScene
from task_trees.behaviours_move import DoMoveNamedPose, DoMoveXYZ, DoRotate, DoMoveXYZRPY
from task_trees.behaviours_base import *
from task_trees.task_trees_manager import BasicTaskTreesManager, TaskTreesManager, BasicTask, TaskStates
from demos.gridscan.behaviours_gridscan import SimCalibrate, DoMoveTankGrid
from demos.gridscan.behaviours_advanced import DoMoveTankGridVisualCDROS

# -------------------------------------
# Tasks specialized for this application 

# A sequence of behaviours for calibrating the pose of the tank 
class CalibrateTask(BasicTask):
    def __init__(self):   
        """ the constructor
        """
        super(CalibrateTask, self).__init__()
    def get_id(self):
        return self.id if hasattr(self, 'id') else None

# A sequence of behaviours for moving the arm to one of the named pose 
class MoveNamedPoseTask(BasicTask):
    def __init__(self, named_pose:str):
        """ the constructor
        :param named_pose: one of the named pose defined in the configuration file
        :type named_pose: str
        """
        if named_pose is None or type(named_pose) is not str or len(named_pose) == 0:
            rospy.logerr(f'{__class__.__name__}: parameter (named_pose) is not an non-empty string -> fix the missing value at behaviour construction')
            raise AssertionError(f'A parameter is invalid')  
        super(MoveNamedPoseTask, self).__init__('named_poses.' + named_pose)
        
class MoveGridPoseTask(BasicTask):
    def __init__(self, tile_x:int, tile_y:int, cell_x:int, cell_y:int): 
        super(MoveGridPoseTask, self).__init__([tile_x, tile_y, cell_x, cell_y])

# ----------------------------------------
# The TaskManager specialized for this application
class GridScanTaskTreesManager(TaskTreesManager):
    """ This is a subclass specialized for the GridScanTaskTreesManager application
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(GridScanTaskTreesManager, self).__init__(arm_commander)
        # load the task scene 
        scene_config_file = os.path.join(os.path.dirname(__file__), 'task_scene.yaml')
        self.the_scene:GridScanScene = GridScanScene(scene_config_file)
        # setup the robotic manipulation platform through the commander
        self._reset_commander()
        # setup simulation end-effector
        self.arm_commander.info(print=True)
        self.arm_commander.add_box_to_scene('instrument', [0.05, 0.05, 0.1], [0, 0, 0.051], [0, 0, 0], reference_frame='tool0')
        self.arm_commander.attach_object_to_end_effector('instrument')
        # setup name poses
        self.named_poses = self.the_scene.keys_config('named_poses')
        for pose_name in self.named_poses:
            pose_name = 'named_poses.' + pose_name
            self.arm_commander.add_named_pose(pose_name, self.the_scene.query_config(pose_name))
        # setup the blackboard and the two blackboard keys 'task'
        self.the_blackboard.register_key(key='task', access=py_trees.common.Access.READ) 
        # build and install the behavior tree
        self._set_initialize_branch(self.create_initialize_branch())
        self._add_priority_branch(self.create_timeout_branch())
        self._add_task_branch(self.create_calibrate_task_branch(), CalibrateTask)
        self._add_task_branch(self.create_move_named_pose_task_branch(), MoveNamedPoseTask)
        self._add_task_branch(self.create_no_tank_trap_branch())
        self._add_task_branch(self.create_move_tank_pose_task_branch(), MoveGridPoseTask)
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)

    # internal function (blocking) for starting the tick_tocking, 
    def _tick_tock(self):
        self.bt.tick_tock(period_ms=self.spin_period_ms)

    # internal function to reset the commander 
    def _reset_commander(self):      
        self.arm_commander.set_max_cartesian_speed(0.1)
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        self.arm_commander.set_workspace_walls(*(self.the_scene.query_config('regions.workspace')))
    
    # return the logical pose of the accepted task, or raise TypeError if no task is submitted
    def query_grid_position_of_task(self):
        """ return the goal, which is a logical pose, of the current task
        :raises TypeError: no task has been submitted
        :return: the logical pose
        :rtype: unspecified as defined by the specific subclass of BasicTask
        """
        if not self.the_blackboard.exists('task'):
            raise TypeError(f'unable to query logical pose due to no task has been submitted')
        return self.the_blackboard.task.get_goal_as_logical_pose()

    # return the logical rotation of the accepted task, or raise TypeError if no task is submitted   
    def query_logical_rpy_of_task(self):
        if not self.the_blackboard.exists('task'):
            raise TypeError(f'unable to query logical rotation due to no task has been submitted')            
        grid_position = self.the_blackboard.task.get_goal_as_logical_pose()   
        rpy = self.the_scene.compute_rpy_from_grid_position(grid_position)
        return rpy

    # internal function for obtaining the logical pose of the current task as xyzrpy
    def _get_task_target_xyzrpy(self) -> list:
        if self.the_blackboard.exists('task'):
            logical_pose = self.the_blackboard.task.get_goal_as_logical_pose()
            return self.the_scene.compute_xyzrpy_from_grid_position(logical_pose)
        return False
    
    # internal function for obtaining the time lapse since the current task was submitted
    def _get_time_since_submit(self) -> float:
        if self.the_blackboard.exists('task'):
            return self.the_blackboard.task.get_time_since_submit()
        # return -1 so that the timeout condition is never True
        return -1        
    
    # --------------------------------------
    # -- internal functions: functions for creating custom constraints for the behaviours 

    # return orientation constraint to fix the rotation of the end_effector during movement in the tank
    def _create_intank_fix_rotate_constraint(self) -> Constraints:
        return moveit_tools.create_path_orientation_constraint(
            self.arm_commander.END_EFFECTOR_LINK, self.arm_commander.pose_in_frame('the_tank'), 0.05, 0.05, 6.28)
    
    # return position constraint to confine the movement in the tank
    def _create_intank_movement_constraint(self) -> Constraints:
        return moveit_tools.create_position_constraint_from_bbox(
           self.arm_commander.END_EFFECTOR_LINK, self.arm_commander.WORLD_REFERENCE_LINK, self.the_scene.query_config('tank.work3d'))
    
    # --------------------------------------
    # -- internal functions: conditions functions for behaviours for the building of the behaviour tree for this TaskManager
    def task_is_timeout(self, duration=30):
        # rospy.loginfo(f'task_timeout: {self.get_time_since_submit()} > {duration}')
        return self._get_time_since_submit() > duration

    def in_a_region(self, region) -> bool:
        current_pose = self.arm_commander.pose_of_robot_link()
        return moveit_tools.in_region(current_pose.pose.position.x, current_pose.pose.position.y, 
                                    self.the_scene.query_config(region))  
    
    def on_or_above_z(self, position) -> bool:
        current_pose = self.arm_commander.pose_of_robot_link()
        return current_pose.pose.position.z > self.the_scene.query_position_as_xyz(position)[2] - 0.05
    
    def below_z(self, position) -> bool:
        current_pose = self.arm_commander.pose_of_robot_link()
        return current_pose.pose.position.z < self.the_scene.query_position_as_xyz(position)[2] - 0.05
    
    def wrong_orientation(self) -> bool:
        target_xyzrpy = self._get_task_target_xyzrpy()
        current_xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy(reference_frame='the_tank')
        # rospy.loginfo(f'same orientation: {target_xyzrpy[3:]} {current_xyzrpy[3:]}')
        return not moveit_tools.same_rpy_with_tolerence(target_xyzrpy[3:], current_xyzrpy[3:], 0.1)

    def wrong_xy_at_tank(self) -> bool:
        target_xyzrpy = self._get_task_target_xyzrpy()
        current_xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy(reference_frame='the_tank')
        # rospy.loginfo(f'wrong_xy_at_tank: target {target_xyzrpy[:2]} current {current_xyzrpy[:2]}')
        return (abs(target_xyzrpy[0] - current_xyzrpy[0]) > 0.01) or (abs(target_xyzrpy[1] - current_xyzrpy[1]) > 0.01)
    
    def at_angle(self, rotation_pose) -> bool:
        current_xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy(reference_frame='the_tank')
        rotation_rpy = self.the_scene.query_rotation_as_rpy(rotation_pose)
        for index in range(len(rotation_rpy)):
            if rotation_rpy[index] is None:
                continue
            if not isclose(current_xyzrpy[3 + index], rotation_rpy[index], abs_tol=0.1):
                return False
        return True
    
    def at_a_named_pose(self, named_pose) -> bool:
        joint_values = self.arm_commander.current_joint_positons_as_list()
        result = moveit_tools.same_joint_values_with_tolerence(self.the_scene.query_config(named_pose), joint_values, 0.05)
        # rospy.loginfo(f'at_home_pose: {result}')
        return result    
    
    # -----------------------------------------------
    # --- internal functions: eternal guard condition
    def guard_work_branch_condition(self) -> bool:
        if self.the_blackboard.exists('task'):
            if self.the_blackboard.task.state == TaskStates.CANCELLED:
                return False
            return True
        return False
    
    # -------------------------------------------------
    # --- create the behaviour tree and its branches
    
    def create_calibrate_task_branch(self) -> Composite:
        """ Create the branch for the CalibrateTask
        :return: The branch
        :rtype: Composite
        """
        calibrate_branch = py_trees.decorators.Timeout(
            duration=60,
            name='calibrate_branch_timeout',
            child=py_trees.composites.Sequence(
                'calibrate_branch',
                memory=True,
                children=[   
                    DoMoveXYZ('move_up_for_calibrate', [{'_fn': self.in_a_region, 'region':'regions.work'}, 
                                                        {'_fn': self.below_z, 'position':'tank.positions.hover'}], 
                                        arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='tank.positions.hover', reference_frame='the_tank', cartesian=True),
                    DoMoveNamedPose('move_home_for_calibrate', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'), 
                    SimCalibrate('do_calibrate', self.arm_commander, scene=self.the_scene),
                ],
            ),
        )
        return calibrate_branch
    
    def create_move_named_pose_task_branch(self) -> Composite:   
        """ Create the branch for the MoveNamedPoseTask
        :return: The branch
        :rtype: Composite
        """ 
        move_named_pose_branch = py_trees.decorators.Timeout(
            duration=60,
            name='move_named_pose_branch_timeout',
            child=py_trees.composites.Sequence(
                'move_named_pose_branch',
                memory=True,
                children=[
                    DoMoveXYZ('move_up_if_in_water', [{'_fn': self.in_a_region, 'region': 'regions.work'}, {'_fn': self.below_z, 'position':'tank.positions.hover'}], 
                                arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='tank.positions.hover',
                                reference_frame='the_tank', cartesian=True),                    
                    DoMoveNamedPose('move_home_first', [{'_fn': self.in_a_region, 'region': 'regions.work'}, 
                                                        {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'}], 
                                    arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),
                    DoMoveNamedPose('move_task_pose', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose=self.query_grid_position_of_task),                            
                    ],
            ),
        )
        return move_named_pose_branch
    
    def create_move_tank_pose_task_branch(self) -> Composite:
        """ Create the branch for the MoveGridPoseTask
        :return: The branch
        :rtype: Composite
        """    
        move_tank_pose_branch = py_trees.decorators.Timeout(
            duration=120,
            name='move_tank_pose_branch_timeout',
            child=py_trees.composites.Sequence(
                'move_tank_pose_branch',
                memory=True,
                children=[
                    DoMoveNamedPose('move_home_if_in_inner_region', {'_fn': self.in_a_region, 'region': 'regions.inner'},
                                    arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'), 
                    DoMoveTankGridVisualCDROS('move_to_work_transition_pose', {'_fn': self.at_a_named_pose, 'named_pose':'named_poses.home'}, 
                                                    arm_commander=self.arm_commander, scene=self.the_scene, grid_position=['tank.grid_positions.transition', 'tank.positions.hover']),
                    DoMoveXYZ('move_up_if_wrong_orientation', [self.wrong_orientation, {'_fn': self.below_z, 'position':'tank.positions.hover'}], 
                              arm_commander=self.arm_commander, scene=self.the_scene, 
                              target_xyz='tank.positions.hover', reference_frame='the_tank', constraint_fn=self._create_intank_fix_rotate_constraint),
                    DoMoveTankGridVisualCDROS('move_transition_pose', [self.wrong_orientation, {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'}], 
                                              arm_commander=self.arm_commander, scene=self.the_scene, grid_position='tank.grid_positions.transition', constraint_fn=self._create_intank_fix_rotate_constraint),
                    DoRotate('rotate_gamma_if_wrong_orientation', [self.wrong_orientation, {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'}], 
                             arm_commander=self.arm_commander, scene=self.the_scene, 
                             target_rpy='tank.rotations.gamma', reference_frame='the_tank'),                   
                    DoRotate('rotate_if_wrong_orientation', [self.wrong_orientation, {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'}], 
                             arm_commander=self.arm_commander, scene=self.the_scene, target_rpy=self.query_logical_rpy_of_task, reference_frame='the_tank'),
                    DoMoveTankGridVisualCDROS('move_xy_if_wrong_position', self.wrong_xy_at_tank, arm_commander=self.arm_commander, scene=self.the_scene, 
                                              grid_position=self.query_grid_position_of_task, constraint_fn=self._create_intank_fix_rotate_constraint),
                    DoRotate('set_oblique_angle', [{'_not_fn': self.at_angle, 'rotation_pose': 'tank.rotations.delta'}, {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'}], 
                             arm_commander=self.arm_commander, scene=self.the_scene, 
                             target_rpy='tank.rotations.delta', reference_frame='the_tank'),
                    DoMoveXYZRPY('submerge_if_hover', [{'_fn': self.at_angle, 'rotation_pose': 'tank.rotations.delta'}, {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'}], 
                                arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='tank.positions.submerged', target_rpy=self.query_logical_rpy_of_task, 
                                reference_frame='the_tank'),                                                    
                    ],
            )
        )
        return move_tank_pose_branch

    def create_no_tank_trap_branch(self) -> Composite:
        """ Create the branch for trapping the situation of no tank is found. This is to prevent the running of the branch for MoveGridPoseTask
        :return: The branch
        :rtype: Composite
        """     
        no_tank_trap = py_trees.composites.Sequence(
                'no_tank_trap_branch',
                memory=True,
                children=[
                    py_trees.behaviours.CheckBlackboardVariableValue(
                        name='check_no_tank_goal', check=py_trees.common.ComparisonExpression(
                            variable='task.name', value=MoveGridPoseTask, operator=operator.eq)),
                    py_trees.decorators.Inverter(name='inverter', child=
                        py_trees.behaviours.CheckBlackboardVariableExists(name='check_tank_exists', variable_name='tank')),
                    py_trees.behaviours.SetBlackboardVariable(name='set_task_success', variable_name='task.state', 
                                                              variable_value=TaskStates.FAILED, overwrite=True),
                    py_trees.behaviours.Success('end_task_if_no_tank')
                ]
        )
        return no_tank_trap
    
    def create_timeout_branch(self) -> Composite:
        """ Create the prioritized branch that restores the robot arm to a more restrained pose after a period of non-activity
        :return: The branch
        :rtype: Composite
        """    
        timeout_branch = py_trees.composites.Sequence(
            'no_task_timeout_branch',
            memory=True,
            children = [
                DoMoveXYZ('move_up_if_timeout', [
                                        {'_fn': self.task_is_timeout, 'duration': 60},
                                        {'_fn': self.in_a_region, 'region': 'regions.work'},
                                        {'_fn': self.below_z, 'position':'tank.positions.hover'},
                                        ], arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='tank.positions.hover'),
                DoMoveNamedPose('move_home_if_timeout',  [
                                        {'_fn': self.task_is_timeout, 'duration': 90},
                                        {'_fn': self.in_a_region, 'region': 'regions.work'}, 
                                        {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'},],
                                        arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),
                DoMoveNamedPose('move_stow_if_timeout', [
                                        {'_fn': self.task_is_timeout, 'duration': 120}, 
                                        {'_fn': self.in_a_region, 'region': 'regions.inner'},
                                        {'_not_fn': self.at_a_named_pose, 'named_pose': 'named_poses.stow'},],
                                        arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.stow'),
            ],
        )
        return timeout_branch
    
    def create_initialize_branch(self) ->Composite:
        """ Create a branch for restoring the robot pose at start up
        :return: The branch
        :rtype: Composite
        """
        init_action_branch = py_trees.composites.Sequence(
            'init_action_branch',
            memory=True,
            children = [
                DoMoveXYZ('move_up_if_in_water', [
                                        {'_fn': self.in_a_region, 'region': 'regions.work'},
                                        {'_fn': self.below_z, 'position':'tank.positions.hover'},
                                        ], arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='tank.positions.hover'),
                DoMoveNamedPose('move_home_if_in_work_region',  [
                                        {'_fn': self.in_a_region, 'region': 'regions.work'}, 
                                        {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'},],
                                        arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),
                DoMoveNamedPose('move_stow_if_in_inner_region', [
                                        {'_fn': self.in_a_region, 'region': 'regions.inner'},
                                        {'_not_fn': self.at_a_named_pose, 'named_pose': 'named_poses.stow'},],
                                        arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.stow'),                
            ]
        )
        return init_action_branch


      


