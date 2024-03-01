# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

from enum import Enum

# the states defined for the Task Manager
class TaskStates(Enum):
    INVALID = -1
    STANDBY = 0
    SUBMITTED = 1
    WORKING = 2
    SUCCEEDED = 3
    CANCELLED = 4  # the eternal guard is alerted and the current commander will be aborted
    ABORTED = 5    # when the comamnder state has changed to ABORTED
    FAILED = 6

# the list of states indicating completion of the task
COMPLETION_STATES = [TaskStates.SUCCEEDED, 
                     TaskStates.ABORTED, 
                     # TaskStates.CANCELLED,
                     TaskStates.FAILED, 
                     TaskStates.INVALID] 

