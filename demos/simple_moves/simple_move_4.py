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

import random
from py_trees.composites import Sequence, Parallel, Composite, Selector

# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
from task_trees.behaviours_move import DoMoveXYZRPY
from task_trees.task_trees_manager import TaskTreesManager

# ---------------------------------------
# The TaskManager specialized for this application
class SimpleMoveTaskManager(TaskTreesManager):
    """ This is a subclass of TaskManager, for illustration of using the framework for developing behaviour trees
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
        super(SimpleMoveTaskManager, self).__init__(arm_commander)

        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()

        # build and install the behavior tree
        self._set_initialize_branch(self.create_init_branch())
        self._add_priority_branch(self.create_move_branch())

        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
    
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
        move_branch = Sequence(
                'move_branch',
                memory=True,
                children=[
                    DoMoveXYZRPY('move_xyzrpy', True, arm_commander=self.arm_commander, 
                                 target_xyz=[[0.3, None, None], self.generate_random_xyz],
                                 target_rpy=self.generate_random_rpy), 
                    ],
        )
        return move_branch
    
    # returns a behaviour tree branch that performs initialzation of the robot by moving to a prescribed pose
    def create_init_branch(self) -> Composite:
        # - the branch that executes the task MoveNamedPoseTask
        init_branch = Sequence(
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
        the_task_manager = SimpleMoveTaskManager(arm_commander)
        # display the behaviour tree as an image
        # the_task_manager.display_tree(target_directory=os.path.dirname(__file__))
        logger.info('simple_move_example is running')
        the_task_manager.spin()
    except Exception as e:
        logger.exception(e)
      


