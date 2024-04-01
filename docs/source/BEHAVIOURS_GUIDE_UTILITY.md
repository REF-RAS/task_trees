# Programming Guide to Utility Behaviours

The utility IUB classes serve the following purposes.
- Output information use for debugging
- Scene manipulation use for simulation

## Debugging IUB Classes

The following behaviour classes can be used in a behaviour tree for printing a message and other information.

### Class task_trees.behavious_base.Print

This class extends `py_trees.behaviour.Behaviour`. It prints a message to the screen when ticked, and after that the tick will move to the next behaviour in the same `Sequence` or `Selector`.
```
class Print(Behaviour):
    def __init__(self, msg, print_tree=False):
    ...
```
The parameter `msg` supports late binding through passing a function. The function should return a string message.

### Class task_trees.behavious_base.PrintPose

This class extends `py_trees.behaviour.Behaviour`. It prints the psoe of the end-effector and the joint values, and after that the tick will move to the next behaviour in the same `Sequence` or `Selector`.

```
class PrintPose(Behaviour):
    def __init__(self, arm_commander:GeneralCommander=None):
    ...
```

### Class task_trees.behavious_base.PrintPoseInFrame

This class extends `task_trees.behavious_base.PrintPose`. It prints the pose of the end-effector in the given reference frame. The parameter `reference_frame` supports late binding through passing a function. The constructor requires the scene configuration model if a custom reference_frame is used.

```
class PrintPosesInFrame(PrintPose):
    def __init__(self, arm_commander:GeneralCommander=None, scene:Scene=None, reference_frame:str=None):
    ...
```

### Example:

The following code snipped is extracted from `simple_move_3_printpose.py` [source code](../../demos/simple_moves/simple_move_3_printpose.py) under `demo/simple_moves`.

```
    def create_move_branch(self) -> Composite:
        """ Returns a behaviour tree branch that move the end_effector to random positions
        :return: a branch for the behaviour tree  
        :rtype: Composite
        """
        move_branch = Sequence(
                'move_branch',
                memory=True,
                children=[
                    Print('Before Random Move', print_tree=True),
                    DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=self.generate_random_xyz), 
                    PrintPose(arm_commander=arm_commander),
                    ],
        )
        return move_branch
```
Notes:
- The `Print` behaviour is specified to print the behaviour tree status in addition to a text message when it is executed. The text message can be replaced by a function reference that can return a message dynamically.
- The `PrintPose` behaviour is used to display the pose after the random move.

## Simulation IUB Classes

The following behaviour classes can modify the robot configuration by attaching or detaching objects to the robot arm. It is useful for simulation such as grabbing and releasing an object in a simulated environment. Both of these classes inherit from `task_trees.behavious_base.ConditionalCommanderBehaviour`

### Class task_trees.behavious_base.SimAttachObject

This class attachs an named object to the end-effector link of the robot. 

```
class SimAttachObject(ConditionalCommanderBehaviour):
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                 object_name=None, post_fn=None):
    ...
```
The parameter `post_fn` allows the definition of a function that is called after the object is attached to the robot.

### Class task_trees.behavious_base.SimDetachObject

This class detach an named object from the end-effector link of the robot. 

```
class SimDetachObject(ConditionalCommanderBehaviour):
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                 object_name=None, post_fn=None, to_remove=False):
    ...
```

If the parameter `to_remove` is `True`, the detached object is also removed from the world. This feature is useful in simulation.



### Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Mar 2024