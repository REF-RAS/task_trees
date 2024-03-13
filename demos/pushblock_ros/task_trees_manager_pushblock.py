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
from task_trees.behaviours_base import SimAttachObject, SimDetachObject
from task_trees.behaviours_move import DoMoveNamedPose, DoMoveXYZ, DoMoveXYZRPY, DoRotate
from task_trees.task_trees_manager import TaskTreesManager, BasicTask
from task_trees.task_scene import Scene

# -------------------------------------
# Tasks specialized for the Pick-n-Drop 

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
        super(MoveNamedPoseTask, self).__init__(named_pose)  

# A sequence of behaviours for moving the arm to one of the named pose 
class PushBlockTask(BasicTask):
    def __init__(self, area:str):
        """ the constructor
        :param area: one of the named pose defined in the configuration file
        :type area: str
        """
        super(PushBlockTask, self).__init__(area)
        if not (area is not None or type(area) is not str or not area.startswith('area_')):
            rospy.logerr(f'{__class__.__name__}: parameter (area) is not a string starts with "area_" -> fix the value at behaviour construction')
            raise AssertionError(f'A parameter is invalid')  

class MoveAreaTask(BasicTask):
    def __init__(self, area:str):
        """ the constructor
        :param area: one of the named pose defined in the configuration file
        :type area: str
        """
        super(MoveAreaTask, self).__init__(area)
        if not (area is not None or type(area) is not str or not area.startswith('area_')):
            rospy.logerr(f'{__class__.__name__}: parameter (area) is not a string starts with "area_" -> fix the value at behaviour construction')
            raise AssertionError(f'A parameter is invalid')  
        
# ----------------------------------------
# The TaskManager specialized for this application
class PushBlockTaskTreesManager(TaskTreesManager):
    """ This is a subclass specialized for the pick-n-drop application
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(PushBlockTaskTreesManager, self).__init__(arm_commander)
        # load the task scene
        config_file = os.path.join(os.path.dirname(__file__), 'task_scene.yaml')
        self.the_scene = Scene(config_file)
        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        # self.arm_commander.set_workspace_walls(*(self.the_scene.query_config('regions.workspace')))
        # setup name poses
        self.named_poses = self.the_scene.keys_config('named_poses')
        for pose_name in self.named_poses:
            pose_name = 'named_poses.' + pose_name
            self.arm_commander.add_named_pose(pose_name, self.the_scene.query_config(pose_name))

        # setup the blackboard and the two blackboard keys 'seen_object'
        self.the_blackboard.register_key(key='the_object', access=py_trees.common.Access.WRITE)      
       
        # build and install the behavior tree
        self._set_initialize_branch(self.create_initialize_branch())
        self._add_task_branch(self.create_move_namedpose_task_branch(), MoveNamedPoseTask)
        self._add_task_branch(self.create_push_block_task_branch(), PushBlockTask)
        
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
    
    # -----------------------------------------
    # Functions for the simulation (demo)
    def setup_objects(self):
        # setup objects
        for object_name in self.the_scene.list_object_names():
            the_object = self.the_scene.get_object_config(object_name)
            if the_object.type == 'box':
                self.arm_commander.add_box_to_scene(object_name, the_object.dimensions, the_object.xyz, the_object.rpy)

    def generate_the_object(self):
        self.the_object = dict()
        self.the_blackboard.the_object = self.the_object
        the_block_config = self.the_scene.query_config('push_block')
        self.the_object['area'] = 'area_1'
        # transform the pose
        xyzrpy = the_block_config['xyz'] + the_block_config['rpy']
        the_pose = moveit_tools.xyzrpy_to_pose_stamped(xyzrpy, self.the_object['area'])
        the_pose = self.arm_commander.transform_pose(the_pose, self.arm_commander.WORLD_REFERENCE_LINK)
        xyzrpy_in_world = moveit_tools.pose_stamped_to_xyzrpy(the_pose)
        self.arm_commander.add_box_to_scene('the_object', the_block_config['dimensions'], xyzrpy_in_world[:3], xyzrpy_in_world[3:])
        
    # -----------------------------------------
    # Functions for the behaviour tree
    def post_fn_update_area(self):
        target = self.query_target_reference_frame()
        self.the_blackboard.the_object['area'] = target 

    # return the logical pose of the accepted task, or raise TypeError if no task is submitted
    def query_logical_pose_of_task(self):
        """ return the goal, which is a logical pose, of the current task
        :raises TypeError: no task has been submitted
        :return: the logical pose
        :rtype: unspecified as defined by the specific subclass of BasicTask
        """
        if self.the_blackboard.exists('task'):
            return self.the_blackboard.task.get_goal_as_logical_pose()
        raise TypeError(f'unable to query logical pose due to no task has been submitted')
    
    def query_current_reference_frame(self):
        if self.the_blackboard.exists('the_object'):
            return self.the_blackboard.the_object['area']
        raise TypeError(f'unable to query the reference frame where the object is located')           
    
    def query_target_reference_frame(self):
        """ return the target area of the task
        :raises TypeError: no task has been submitted
        :return: the reference frame representing the target area
        :rtype: str
        """
        if self.the_blackboard.exists('task'):
            target = self.the_blackboard.task.get_goal_as_logical_pose()
            return target
        raise TypeError(f'unable to query the target due to no task has been submitted')                

    # return the ideal rotation of the end-effector for pushing to the target, which is derived from the target and the structure of the end-effector
    # defined in the config file
    def query_rotation_from_target(self):
        if self.the_blackboard.exists('task'):
            target = self.the_blackboard.task.get_goal_as_logical_pose()
            rotation = self.the_scene.query_config(target + ".rotation")
            return rotation
        raise TypeError(f'unable to query the rotation due to no task has been submitted or the target area is wrong')         
     
    # return the ideal rotation of the end-effector for pushing from the source, which is derived from the source and the structure of the end-effector
    # defined in the config file        
    def query_rotation_from_current(self):
        if self.the_blackboard.exists('the_object'):
            current = self.the_blackboard.the_object['area']
            rotation = self.the_scene.query_config(current + ".rotation")
            return rotation
        raise TypeError(f'unable to query the rotation due to no task has been submitted or the target area is wrong')         
        
    # internal function for obtaining the time lapse since the current task was submitted
    def _get_time_since_submit(self) -> float:
        if self.the_blackboard.exists('task'):
            return self.the_blackboard.task.get_time_since_submit()
        # return -1 so that the timeout condition is never True
        return -1        
    
    # return orientation constraint to fix the rotation of the end_effector during movement in the tank
    def _create_intank_fix_rotate_constraint(self):
        return moveit_tools.create_path_orientation_constraint(
            self.arm_commander.END_EFFECTOR_LINK, self.arm_commander.pose_in_frame(), 0.05, 0.05, 0.05)
    
    # --------------------------------------
    # -- internal functions: conditions functions for behaviours for the building of the behaviour tree for this TaskManager
    def task_is_timeout(self, duration=30):
        # rospy.loginfo(f'task_timeout: {self.get_time_since_submit()} > {duration}')
        return self._get_time_since_submit() > duration

    def in_a_region(self, logical_region) -> bool:
        current_pose = self.arm_commander.pose_of_robot_link()
        return moveit_tools.in_region(current_pose.pose.position.x, current_pose.pose.position.y, self.the_scene.query_config(logical_region))     
    
    def over_an_object(self, object_name) -> bool:
        the_object = self.the_scene.get_object_config(object_name)
        the_object_position_as_bbox = [the_object.xyz[0] - the_object.dimensions[0] / 2, the_object.xyz[1] - the_object.dimensions[1] / 2,
                                    the_object.xyz[0] + the_object.dimensions[0] / 2, the_object.xyz[1] + the_object.dimensions[1] / 2]
        current_pose = self.arm_commander.pose_of_robot_link()
        return moveit_tools.in_region(current_pose.pose.position.x, current_pose.pose.position.y, the_object_position_as_bbox)   
     
    def on_or_above_z(self, position) -> bool:
        current_pose = self.arm_commander.pose_of_robot_link()
        return current_pose.pose.position.z > self.the_scene.query_position_as_xyz(position)[2] - 0.05
    
    def below_z(self, position) -> bool:
        current_pose = self.arm_commander.pose_of_robot_link()
        return current_pose.pose.position.z < self.the_scene.query_position_as_xyz(position)[2] - 0.05
        
    def wrong_restart_position(self) -> bool:
        target = self.query_target_reference_frame()
        start_position = self.the_scene.query_config('area.positions.restart')
        current_pose_in_target_frame = self.arm_commander.pose_in_frame_as_xyzrpy(reference_frame=target)
        distance = math.dist(start_position[:2], current_pose_in_target_frame[:2])
        # rospy.loginfo(f'wrong_restart_position: {target} {start_position} {current_pose_in_target_frame} {distance}')
        return distance > 0.02
    
    def the_object_in_area(self, area) -> bool:
        if self.the_blackboard.exists('the_object'):
            if 'area' in self.the_blackboard.the_object:
                current = self.the_blackboard.the_object['area']
                return current == area
        rospy.logwarn(f'the_object_in_area: the blackboard key "the_object" is not defined or it has no "area" key -> should not happen')
        return False        
    
    def the_object_over_an_object(self, object_name) -> bool:
        the_target_object = self.the_scene.get_object_config(object_name)
        the_target_position_as_bbox = [the_target_object.xyz[0] - the_target_object.dimensions[0] / 2, the_target_object.xyz[1] - the_target_object.dimensions[1] / 2,
                                    the_target_object.xyz[0] + the_target_object.dimensions[0] / 2, the_target_object.xyz[1] + the_target_object.dimensions[1] / 2]
        xyzrpy_of_the_object = self.arm_commander.pose_in_frame_as_xyzrpy('the_object')
        return moveit_tools.in_region(xyzrpy_of_the_object[0], xyzrpy_of_the_object[1], the_target_position_as_bbox)           
            
    def at_a_named_pose(self, named_pose) -> bool:
        joint_values = self.arm_commander.current_joint_positons_as_list()
        result = moveit_tools.same_joint_values_with_tolerence(self.the_scene.query_config(named_pose), joint_values, 0.05)
        # rospy.loginfo(f'at {named_pose} pose: {result}')
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
    
    def create_move_namedpose_task_branch(self) -> Composite:
        # - the branch that executes the task MoveNamedPoseTask
        move_named_pose_branch = py_trees.decorators.Timeout(
            duration=60,
            name='move_named_pose_branch_timeout',
            child=py_trees.composites.Sequence(
                'move_named_pose_branch',
                memory=True,
                children=[
                    DoMoveNamedPose('move_home_first', [{'_not_fn': self.in_a_region, 'logical_region': 'regions.inner'}], 
                                    arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),
                    DoMoveNamedPose('move_task_pose', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose=self.query_logical_pose_of_task),                    
                    ],
            ),
        )
        return move_named_pose_branch
    
    def create_push_block_task_branch(self) -> Composite:
        # - the branch that executes the task ScanTask
        move_area_branch = py_trees.decorators.Timeout(
            duration=1000000,
            name='scan_branch_timeout',
            child=py_trees.composites.Sequence(
                'scan_branch',
                memory=True,
                children=[
                    DoMoveNamedPose('move_home_first_if_inner', [{'_fn': self.in_a_region, 'logical_region': 'regions.inner'}], 
                                    arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),
                    DoMoveXYZ('move_to_area_edge', [{'_not_fn': self.the_object_in_area, 'area':'area_5'}], arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['area.positions.start', 'area.positions.hover'], 
                                                    reference_frame=self.query_current_reference_frame), 
                    DoRotate('align_with_the_alley', [{'_not_fn': self.the_object_in_area, 'area':'area_5'}], arm_commander=self.arm_commander, scene=self.the_scene,
                                                target_rpy=self.query_rotation_from_current, reference_frame=self.query_current_reference_frame),
                    DoMoveXYZ('move_down', [{'_not_fn': self.the_object_in_area, 'area':'area_5'}], arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['area.positions.down'], cartesian=True,
                                                    reference_frame=self.query_current_reference_frame),  
                    SimAttachObject('engage_object', True, arm_commander=self.arm_commander, object_name='the_object'),    
                    DoMoveXYZ('push_object_to_area_5', [{'_not_fn': self.the_object_in_area, 'area':'area_5'}], arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['area.positions.centre'], cartesian=True,
                                                    reference_frame=self.query_current_reference_frame),    
                    SimDetachObject('detach_at_centre', True, arm_commander=self.arm_commander, object_name='the_object', post_fn=lambda: 'area_5'), 
                    DoMoveXYZ('move_up', {'_fn': self.wrong_restart_position}, arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['area.positions.hover'], 
                                                    reference_frame=self.query_current_reference_frame),
                    DoMoveXYZRPY('move_align_for_restart', {'_fn': self.wrong_restart_position}, arm_commander=self.arm_commander, scene=self.the_scene,
                                                    target_xyz=['area.positions.restart', 'area.positions.hover'], target_rpy=self.query_rotation_from_target,
                                                    reference_frame=self.query_target_reference_frame),                    
                    DoMoveXYZ('move_down', {'_fn': self.on_or_above_z, 'position':'area.positions.hover'}, arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['area.positions.down'], cartesian=True,
                                                    reference_frame=self.query_target_reference_frame),                     
                    SimAttachObject('touch_object', True, arm_commander=self.arm_commander, object_name='the_object'),
                    DoMoveXYZ('push_object_to_target', True, arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['area.positions.end'], cartesian=True,
                                                    constraint_fn = self._create_intank_fix_rotate_constraint,
                                                    reference_frame=self.query_target_reference_frame), 
                    SimDetachObject('detach_object', True, arm_commander=self.arm_commander, object_name='the_object', post_fn=self.post_fn_update_area),          
                    DoMoveXYZ('move_up', True, arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['area.positions.hover'], 
                                                    reference_frame=self.query_target_reference_frame),                   
                    ],
            ),
        ) 
        return move_area_branch
    

    
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

      


