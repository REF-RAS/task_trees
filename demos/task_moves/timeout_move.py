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

import random, time
from py_trees.composites import Sequence, Parallel, Composite, Selector
from py_trees.decorators import Timeout
# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
from task_trees.behaviours_move import DoMoveXYZ, DoMoveJointPose
from task_trees.task_trees_manager import TaskTreesManager

# ---------------------------------------
# The TaskManager specialized for this application
class TimeoutMoveTaskManager(TaskTreesManager):
    """ This is a subclass of TaskTreesManager, illustrating a timeout priority branch that get activated subject
        to timeout
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(TimeoutMoveTaskManager, self).__init__(arm_commander)

        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()
        # build and install the behavior tree
        self._set_initialize_branch(self.create_init_branch())
        self._add_priority_branch(self.create_timeout_branch())   
        # set timer for the timeout test
        self.start_time = time.time()   
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)
        
    # -------------------------------------------------
    # --- create the behaviour tree and its branches
        
    # --------------------------------------
    # -- internal functions: conditions functions for behaviours for the building of the behaviour tree for this TaskManager
    def is_timeout(self, duration=30):
        return time.time() - self.start_time > duration

    def create_timeout_branch(self) -> Composite:
        """ Returns a behaviour tree branch that is activated after 15 seconds and move back to the reset pose
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        timeout_branch = Sequence('move_branch', memory=True,
                children=[
                    DoMoveJointPose('reset_pose', {'_fn': self.is_timeout, 'duration': 15},
                                    arm_commander=self.arm_commander, 
                                    target_joint_pose=[0.00, -1.243, 0.00, -2.949, 0.00, 1.704, 0.785],), 
                    ],)
        return timeout_branch
    
    def create_init_branch(self) -> Composite:
        """ Returns a behaviour tree branch that moves to a random position
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        init_branch = Sequence('init_branch', memory=True,
                children=[
                    DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=
                        lambda: [random.uniform(0.1, 0.5), random.uniform(-0.3, 0.3), random.uniform(0.2, 0.6)]), 
                    ],
        )
        return init_branch   
    
if __name__=='__main__':
    # rospy.init_node('simple_move_example', anonymous=False)
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_task_manager = TimeoutMoveTaskManager(arm_commander)
        # display the behaviour tree as an image
        # the_task_manager.display_tree(target_directory=os.path.dirname(__file__))
        logger.info('estop_move_example is running')
        the_task_manager.spin()
    except Exception as e:
        logger.exception(e)
      


