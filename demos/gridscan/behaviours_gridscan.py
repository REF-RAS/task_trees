# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

# robot control module
from task_trees.behaviours_move import SceneConditionalCommanderBehaviour
from task_scene_gridscan import GridScanScene
from task_trees.tools import logger

# ----------------------------------------------------------------------
# Custom Behaviour Classes

class SimCalibrate(Behaviour):
    """ This behaviour performs a (simulated) calibration that determines the pose of the tank
    """
    def __init__(self, name, arm_commander=None, scene=None):
        """ the constructor
        :param name: behaviour name
        :type name: str
        :param arm_commander: the general commander to use in this behavour, defaults to None
        :type arm_commander: GeneralCommander
        :param scene: the scene for defining the tank configurations
        :type scene: Scene        
        """
        super(SimCalibrate, self).__init__(name)
        if arm_commander is None or scene is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (arm_commander or scene) is None -> fix the missing value')
            raise AssertionError(f'A parameter should not be None nor missing')
        # define the three key-value pairs used by this behaviour
        self.the_blackboard = py_trees.blackboard.Client()
        self.the_blackboard.register_key(key='task', access=py_trees.common.Access.READ)
        self.the_blackboard.register_key(key='tank', access=py_trees.common.Access.WRITE)        
        # attach robot agent
        self.arm_commander = arm_commander
        self.the_scene = scene

    # the concrete implementation that contains the logic of the simulated calibration
    def update(self):
        logger.info(f'SimCalibrate: found tank at pose: ')  
        # setup objects
        for object_name in self.the_scene.list_object_names():
            the_object = self.the_scene.get_object_config(object_name)
            if the_object.type == 'box':
                self.arm_commander.add_box_to_scene(object_name, the_object.dimensions, the_object.xyz, the_object.rpy)  
            if the_object.name == 'the_tank':
                self.the_blackboard.set('tank', the_object)
        self.the_blackboard.task.result = '12'
        return Status.SUCCESS

class DoMoveTankGrid(SceneConditionalCommanderBehaviour):
    """ This behaviour moves the end-effector to a pose defined in the frame of the work area in a cartesian manner. 
        The roll and pitch components of the orientation of the end-effector is fixed.
    """
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                scene:GridScanScene=None, grid_position=None, constraint_fn=None):
        """ the constructor, refers to the constructor ConditionalCommanderBehaviour for the description of the other parameters
        :param the_scene: the scene model for the handling of logical positions specified in the target xyz
        :type the_scene: Scene
        :param xyz: the target position, which can be specified in a 
        :type xyz: a string representing the z_level_pose as defined in the task scene
        :param reference_frame: the reference_frame
        :type reference_frame: a string representing the reference_frame      
        """
        super(DoMoveTankGrid, self).__init__(name=name, condition_fn=condition_fn, condition_policy=condition_policy, arm_commander=arm_commander, scene=scene, 
                                             reference_frame='the_tank')
        if grid_position is None:
            logger.error(f'{__class__.__name__} ({self.name}): parameter (physical_xyz) is None -> fix the missing value')
            raise AssertionError(f'A parameter should not be None nor missing')
        self.grid_position = grid_position
        self.the_scene:GridScanScene = scene
        self.cartesian = True
        self.constraint_fn = constraint_fn

    # the concrete implementation of the logic when the General Commander is READY        
    def update_when_ready(self):
        # evaluate physical target xyz
        xyz = self.compute_physical_target(self.grid_position, self.the_scene.query_position_as_xyz)  
        if self.grid_position is None or xyz is None:
            logger.error(f'DoMoveTankGrid ({self.name}): invalid position xyz parameter {xyz}')
            return Status.FAILURE 
        # add constraint returned by constraint_fn is given
        if self.constraint_fn is not None and hasattr(self.constraint_fn, '__call__'):
            self.arm_commander.add_path_constraints(self.constraint_fn())
        # send commands  
        self.arm_commander.move_to_position(x=xyz[0], y=xyz[1], z=xyz[2], cartesian=self.cartesian, reference_frame=self.reference_frame, wait=False)
        logger.info(f'DoMoveTankGrid ({self.name}): started move to pose: {xyz} in reference frame "{self.reference_frame}"')   
        return Status.RUNNING
    # the concrete implementation of the logic when the command is completed    
    def tidy_up(self):
        super().tidy_up()
        self.arm_commander.clear_path_constraints()
        