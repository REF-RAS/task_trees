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

import operator, os, random, random

import py_trees
from py_trees.composites import Sequence, Parallel, Composite, Selector

# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
import arm_commander.moveit_tools as moveit_tools

from task_trees.behaviours_base import SimAttachObject, SimDetachObject
from task_trees.behaviours_move import DoMoveNamedPose, DoMoveXYZ
from task_trees.task_trees_manager import GuardedTaskTreesManager, BasicTask
from task_trees.task_scene import Scene

from demos.pickndrop.behaviours_pnd import DoScanProgram
from scan_model import SingleLineScanModel, SteppingScanModel, FourCornersScanModel

# -------------------------------------
# Tasks specialized for the Pick-n-Drop 

# A sequence of behaviours for calibrating the pose of the tank 
class ScanTask(BasicTask):
    def __init__(self):   
        """ the constructor
        """
        super(ScanTask, self).__init__()

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

class PickUpObjectTask(BasicTask):
    def __init__(self, xyz_world:list): 
        super(PickUpObjectTask, self).__init__()
        self.xyz = xyz_world
        
        
class DropObjectTask(BasicTask):
    def __init__(self): 
        super(DropObjectTask, self).__init__()
        
               
class MovePoseTask(BasicTask):
    def __init__(self, xyz_world:list): 
        super(MovePoseTask, self).__init__()
        self.xyz = xyz_world        

# ----------------------------------------
# The TaskManager specialized for this application
class PNDTaskTreesManager(GuardedTaskTreesManager):
    """ This is a subclass specialized for the pick-n-drop application
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(PNDTaskTreesManager, self).__init__(arm_commander)
        # load the task scene
        config_file = os.path.join(os.path.dirname(__file__), 'task_scene.yaml')
        self.the_scene = Scene(config_file)
        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        self.arm_commander.set_workspace_walls(*(self.the_scene.query_config('regions.workspace')))
        # setup name poses
        self._define_named_poses(self.the_scene)

        # setup the blackboard and the two blackboard keys 'seen_object'
        self.the_blackboard.register_key(key='seen_object', access=py_trees.common.Access.WRITE)      
        # setup the scan program
        self.default_scan_program = SingleLineScanModel(self.the_scene.query_config('the_table.regions.scan'), 
                                                    self.the_scene.query_config('the_table.positions.scan_z_level')[2],
                                            self.the_scene.query_config('the_table.scan_step_size'))
        # setup the simulation of ball pick-up
        self.the_ball = None
        
        # build and install the behavior tree
        self._set_initialize_branch(self.create_initialize_branch())
        # self._add_priority_branch(self.create_timeout_branch())
        self._add_task_branch(self.create_move_namedpose_task_branch(), MoveNamedPoseTask)
        self._add_task_branch(self.create_scan_task_branch(), ScanTask)
        self._add_task_branch(self.create_pickup_object_task_branch(), PickUpObjectTask)
        self._add_task_branch(self.create_drop_object_task_branch(), DropObjectTask)
        self._add_task_branch(self.create_move_pose_task_branch(), MovePoseTask)        
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
                        
    # Generate a new ball on the table
    def generate_a_ball(self):
        the_table = self.the_scene.get_object_config('the_table')
        x, y = the_table.dimensions[0] * random.random() * 0.9, the_table.dimensions[1] * random.random() * 0.9 # position in the box frame
        z = the_table.xyz[2] + the_table.dimensions[2] / 2 + 0.035
        xyz = [x, y, z]
        xyz_in_world_frame = [x + the_table.xyz[0] - the_table.dimensions[0] / 2, y + the_table.xyz[1] - the_table.dimensions[1] / 2, z]
        self.arm_commander.add_sphere_to_scene('the_ball', 0.03, xyz_in_world_frame)
        self.the_ball = dict()
        self.the_ball['xyz'] = xyz
        self.the_ball['xyz_world'] = xyz_in_world_frame
        return True
    
    # Simulate the operation of a camera, which looks at an area on the table and if the object is found, write
    # the position of the target to the blackboard key 'seen_object'
    def simulate_camera(self):
        xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy()
        # logger.info(f'arm: {xyzrpy[:2]} ball: {self.the_ball["xyz_world"][:2]}')
        if self.the_ball is None:
            self.the_blackboard.unset('seen_object')
            return
        # logger.info(f'{abs(xyzrpy[0] - self.the_ball["xyz_world"][0])} {abs(xyzrpy[1] - self.the_ball["xyz_world"][1])}')
        if (abs(xyzrpy[0] - self.the_ball['xyz_world'][0]) <= 0.05) and (abs(xyzrpy[1] - self.the_ball['xyz_world'][1]) <= 0.10):
            logger.info(f'simulate_camera: found object at {self.the_ball["xyz_world"]}')
            self.the_blackboard.seen_object = self.the_ball['xyz_world']
        else:
            self.the_blackboard.unset('seen_object')
    
    # -----------------------------------------
    # Functions for the behaviour tree
    
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
    
    # return the target position of the accepted task, or raise TypeError if no task is submitted
    def query_physical_pose_of_task_as_xyz(self):
        """ return the goal, which is a physical pose of xyz, of the current task
        :raises TypeError: no task has been submitted
        :return: xyz
        :rtype: a list of 3 numbers
        """
        if self.the_blackboard.exists('task'):
            xyz = self.the_blackboard.task.get_xyz()
            return xyz
        raise TypeError(f'unable to query the physical pose due to no task has been submitted')

    # return the ideal position of the end-effector for grabbing, which is derived from the target position and the structure of the end-effector
    # defined in the config file
    def query_grab_position_as_xyz(self):
        xyz = self.query_physical_pose_of_task_as_xyz()  
        gripper_offset = self.the_scene.query_config('gripper_offset')
        if gripper_offset is None:
            return xyz
        return list( map(operator.add, xyz, gripper_offset) )
    
    # return the preparatory position of the end-effector for grabbing, which is derived from the target position and the structure of the end-effector
    # defined in the config file, and the preparatory position is directly above the object for a vertical movement
    def query_preparatory_position_as_xyz(self):
        xyz = self.query_physical_pose_of_task_as_xyz()  
        gripper_offset = self.the_scene.query_config('grip_prepare_offset')
        if gripper_offset is None:
            return xyz
        return list( map(operator.add, xyz, gripper_offset) )    
        
    # internal function for obtaining the time lapse since the current task was submitted
    def _get_time_since_submit(self) -> float:
        if self.the_blackboard.exists('task'):
            return self.the_blackboard.task.get_time_since_submit()
        # return -1 so that the timeout condition is never True
        return -1        
    
    # --------------------------------------
    # -- internal functions: conditions functions for behaviours for the building of the behaviour tree for this TaskManager
    def task_is_timeout(self, duration=30):
        # logger.info(f'task_timeout: {self.get_time_since_submit()} > {duration}')
        return self._get_time_since_submit() > duration

    def in_a_region(self, logical_region) -> bool:
        current_pose = self.arm_commander.pose_of_robot_link()
        return moveit_tools.in_region(current_pose.pose.position.x, current_pose.pose.position.y, 
                                    self.the_scene.query_config(logical_region)) 
    
    def over_the_table(self) -> bool:
        the_table = self.the_scene.get_object_config('the_table')
        the_table_position_as_bbox = [the_table.xyz[0] - the_table.dimensions[0] / 2, the_table.xyz[1] - the_table.dimensions[1] / 2,
                                    the_table.xyz[0] + the_table.dimensions[0] / 2, the_table.xyz[1] + the_table.dimensions[1] / 2]
        current_pose = self.arm_commander.pose_of_robot_link()
        return moveit_tools.in_region(current_pose.pose.position.x, current_pose.pose.position.y, the_table_position_as_bbox)         
            
    def at_a_named_pose(self, named_pose) -> bool:
        joint_values = self.arm_commander.current_joint_positons_as_list()
        result = moveit_tools.same_joint_values_with_tolerence(self.the_scene.query_config(named_pose), joint_values, 0.05)
        # logger.info(f'at {named_pose} pose: {result}')
        return result    
    
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
    
    def create_scan_task_branch(self) -> Composite:
        # - the branch that executes the task ScanTask
        scan_branch = py_trees.decorators.Timeout(
            duration=1000000,
            name='scan_branch_timeout',
            child=py_trees.composites.Sequence(
                'scan_branch',
                memory=True,
                children=[
                    DoMoveNamedPose('move_home_first_if_inner', [{'_fn': self.in_a_region, 'logical_region': 'regions.inner'}], 
                                    arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),
                    DoMoveXYZ('move_to_scan_start_pose', {'_fn': self.at_a_named_pose, 'named_pose':'named_poses.home'}, 
                                                    arm_commander=self.arm_commander, scene=self.the_scene, 
                                                    target_xyz=['the_table.positions.start', 'the_table.positions.scan_z_level'], reference_frame='the_table'), 
                    py_trees.decorators.SuccessIsRunning ('scan_program_branch',
                            child=DoScanProgram('scan_program', True, arm_commander=self.arm_commander, scan_program=self.default_scan_program)),
                    ],
            ),
        ) 
        return scan_branch
    
    def create_pickup_object_task_branch(self) -> Composite:
        # - the branch that executes the task PickUpObjectTask
        pick_object_branch = py_trees.decorators.Timeout(
            duration=120,
            name='pickup_object_branch_timeout',
            child=py_trees.composites.Sequence(
                'pickup_object_branch',
                memory=True,
                children=[
                    DoMoveXYZ('move_to_above_object', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz=self.query_preparatory_position_as_xyz),                     
                    DoMoveXYZ('move_to_object', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz=self.query_grab_position_as_xyz),     
                    SimAttachObject('grab_object',arm_commander=self.arm_commander, object_name='the_ball'),
                    ],
            ),
        )
        return pick_object_branch
    
    def create_drop_object_task_branch(self) -> Composite:
        # - the branch that executes the task DropObjectTask   
        drop_object_branch = py_trees.decorators.Timeout(
            duration=120,
            name='drop_object_branch_timeout',
            child=py_trees.composites.Sequence(
                'drop_object_branch',
                memory=True,
                children=[
                    DoMoveNamedPose('move_home_first_if_inner', True, arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),   
                    DoMoveXYZ('move_to_bin_pose', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='positions.drop'), 
                    SimDetachObject('drop_object', arm_commander=self.arm_commander, object_name='the_ball', to_remove=True),                         
                    ],
            ),
        )
        return drop_object_branch
    
    def create_move_pose_task_branch(self) -> Composite:    
        # - the branch that executes the task MovePoseTask (not used in the simulation)
        move_pose_branch = py_trees.decorators.Timeout(
            duration=120,
            name='move_pose_branch_timeout',
            child=py_trees.composites.Sequence(
                'move_pose_branch',
                memory=True,
                children=[
                    DoMoveXYZ('move_to_physical_pose', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz=self.query_physical_pose_of_task_as_xyz),                         
                    ],
            ),
        )
        return move_pose_branch
    
    def create_initialize_branch(self) -> py_trees.decorators.OneShot:
        init_action_branch = py_trees.composites.Sequence(
            'init_action_branch',
            memory=True,
            children = [
                DoMoveNamedPose('move_home', True,
                                        arm_commander=self.arm_commander, scene=self.the_scene, named_pose='named_poses.home'),               
            ]
        )
        return init_action_branch

      


