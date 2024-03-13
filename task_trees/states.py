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
    SUCCEEDED = 3       # the task is successfully completed
    CANCELLED = 4       # the task is being cancelled
    ABORTED = 5         # a cancelled task has successfull abort the commander (state changed to ABORTED)
    FAILED = 6          # an error has occured in the commander
    GUARD_ABORTED = 7   # the eternal guard is alerted and the current task is aborted  

# the list of states indicating completion of the task
COMPLETION_STATES = [TaskStates.SUCCEEDED, 
                     TaskStates.ABORTED, 
                     # TaskStates.CANCELLED,    # the task is not considered completed in the cancel state
                     TaskStates.GUARD_ABORTED,
                     TaskStates.FAILED, 
                     TaskStates.INVALID] 

