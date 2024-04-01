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

import sys, signal

# robot control module
from arm_commander.commander_moveit import GeneralCommander, logger
from behaviours_gridscan import *
from task_trees_manager_gridscan import *

# -- Test cases specialized for the Task Trees Manager

class GridScanDemoApplication():
    """ The application program for the Grid Scanning Demo 
    """
    def __init__(self):
        signal.signal(signal.SIGINT, self.stop)
        self.the_task_manager = GridScanTaskTreesManager(GeneralCommander('manipulator'))
        self._test_main()

    def stop(self, *args, **kwargs):
        logger.info('stop signal received')
        self.the_task_manager.shutdown()
        sys.exit(0)
    
    def _test_main(self):
        self._test_1()

    def _test_1(self):
        task_manager = self.the_task_manager
        
        logger.info(f'=== Submit a MoveNamedPoseTask to stow')
        the_task = MoveNamedPoseTask('stow')
        task_manager.submit_task(the_task)
        the_task.wait_for_completion()
        
        logger.info(f'=== Submit a CalibrateTask')
        the_task = CalibrateTask()
        task_manager.submit_task(the_task)
        the_task.wait_for_completion()

        grid_y = 0
        for grid_x in range(0, 30):
            tile_x, tile_y = grid_x // 6, grid_y // 6
            cell_x, cell_y = grid_x % 6, grid_y % 6
            logger.info(f'=== Submit a MoveGridPoseTask to {tile_x} {tile_y} {cell_x} {cell_y}')
            the_task = MoveGridPoseTask(tile_x, tile_y, cell_x, cell_y)
            task_manager.submit_task(the_task)
            the_task.wait_for_completion()       
        grid_y = 1
        for grid_x in range(29, -1, -1):
            tile_x, tile_y = grid_x // 6, grid_y // 6
            cell_x, cell_y = grid_x % 6, grid_y % 6
            logger.info(f'=== Submit a MoveGridPoseTask to {tile_x} {tile_y} {cell_x} {cell_y}')
            the_task = MoveGridPoseTask(tile_x, tile_y, cell_x, cell_y)
            task_manager.submit_task(the_task)
            the_task.wait_for_completion()                     

        logger.info(f'=== Submit a MoveNamedPoseTask to stow')
        the_task = MoveNamedPoseTask('stow')
        task_manager.submit_task(the_task)
        the_task.wait_for_completion()
        logger.info(f'=== Test Finished')

if __name__=='__main__':
    try:
        logger.info('The task_manager_demo is running')
        GridScanDemoApplication()
    except Exception as e:
        logger.exception(e)


