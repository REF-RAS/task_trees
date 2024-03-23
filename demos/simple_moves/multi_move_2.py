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
from task_trees.behaviours_move import DoMovePose, DoMoveMultiPose
from task_trees.task_trees_manager import TaskTreesManager

# The TaskManager specialized for this application
class MultiMoveTaskManager(TaskTreesManager):
    """ This is a subclass of TaskManager, for illustration of using the framework for developing behaviour trees
        The behaviour tree contains three behaviors, the first move to a constant start pose, the second makes a multi waypoint movement that mimic
        a scan pattern, and the final makes some random rotations that mimic end-effector work
    """
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        """ the constructor
        :param arm_commander: a general commander for a particular arm manipulator 
        :type arm_commander: GeneralCommander
        :param spin_period_ms: the tick_tock period, defaults to 10 seconds
        :type spin_period_ms: int, optional
        """
        super(MultiMoveTaskManager, self).__init__(arm_commander)

        # setup the robotic manipulation platform through the commander
        self.arm_commander.abort_move(wait=True)
        self.arm_commander.reset_world()

        # build and install the behavior tree
        self._add_priority_branch(self.create_move_branch())
        
        # install and unleash the behaviour tree
        self._install_bt_and_spin(self.bt, spin_period_ms)

    # --------------------------------------------------
    # --- functions for behaviour trees to generate multi waypoint poses
    SCAN_PATTERN = [(1, 0.3), (1, -0.6), (2, 0.05), 
                    (1, 0.6), (1, -0.6), (2, 0.05), 
                    (1, 0.6), (1, -0.3)]
    def generate_scan_movement(self) -> list:
        """Returns a list of xyzrpy poses that minic a scan pattern based the current position as the start 

        :return: a list of xyzrpy poses
        :rtype: list
        """
        xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy()
        target_poses = [xyzrpy]
        # change the xyzrpy pose according to the SCAN_PATTERN, of which each element comprises (the index to change, the change value)
        for s in MultiMoveTaskManager.SCAN_PATTERN:
            xyzrpy = xyzrpy.copy()
            xyzrpy[s[0]] += s[1]
            target_poses.append(xyzrpy)
        logger.info(f'generate_scan_movement: {xyzrpy}')
        return target_poses
    
    def generate_random_rotations(self) -> list:
        """Returns a list of xyzrpy poses that minic multiple rotations

        :return: a list of xyzrpy poses
        :rtype: list
        """
        xyzrpy = self.arm_commander.pose_in_frame_as_xyzrpy()
        target_poses = [xyzrpy]
        # change the yaw component of the xyzrpy pose randomly
        for i in range(10):
            xyzrpy = xyzrpy.copy()
            xyzrpy[5] += random.uniform(-1.57, 1.57)
            target_poses.append(xyzrpy)
        logger.info(f'generate_random_rotations: {xyzrpy}')
        return target_poses

    # -------------------------------------------------
    # --- create the behaviour tree and its branches
        
    # A behaviour tree branch that moves to a specified position xyz
    def create_move_branch(self) -> Composite:
        """ Return a behaviour tree branch that comprises three behaviors, the first move to a constant start pose, 
            the second makes a multi waypoint movement that mimic a scan pattern, and the final makes some random rotations that mimic end-effector work
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = Sequence(
                'move_branch',
                memory=True,
                children=[
                    DoMovePose('move_start_pose', True, arm_commander=self.arm_commander, target_pose=[0.3, 0.0, 0.3, 3.14, 0, 2.3]), 
                    DoMoveMultiPose('scan_move_pattern', True, arm_commander=self.arm_commander, target_poses=self.generate_scan_movement), 
                    DoMoveMultiPose('multi_rotations', True, arm_commander=self.arm_commander, target_poses=self.generate_random_rotations),                     
                    ],
        )
        return move_branch
    
if __name__=='__main__':
    # rospy.init_node('simple_move_example', anonymous=False)
    try:
        arm_commander = GeneralCommander('panda_arm')
        the_task_manager = MultiMoveTaskManager(arm_commander)
        # display the behaviour tree as an image
        # the_task_manager.display_tree(target_directory=os.path.dirname(__file__))
        logger.info(f'simple_move_example is running')
        the_task_manager.spin()
    except Exception as e:
        logger.exception(e)
      


