# Task Trees SDK: The Utility Behaviour Classes

The utility behaviour classes in the extension behaviour package can be further divided into 3 types.
- Output information use for debugging
- Scene manipulation use for simulation
- Internal use for task management

## Output information use for debugging

The following behaviour classes can be used in a behaviour tree for printing a message and other information.

### Class task_trees.behavious_base.Print

##### extends py_trees.behaviour.Behaviour

This class prints a message to the screen when ticked, and after that the tick will move to the next behaviour in the same `Sequence` or `Selector`.
```
class Print(Behaviour):
    def __init__(self, msg, print_tree=False):
    ...
```
The parameter `msg` supports late binding through passing a function. The function should return a string message.

### Class task_trees.behavious_base.PrintPose

##### extends py_trees.behaviour.Behaviour

This class prints the psoe of the end-effector and the joint values, and after that the tick will move to the next behaviour in the same `Sequence` or `Selector`.

```
class PrintPose(Behaviour):
    def __init__(self, arm_commander:GeneralCommander=None):
    ...
```

### Class task_trees.behavious_base.PrintPoseInFrame

##### extends task_trees.behavious_base.PrintPose

This class prints the pose of the end-effector in the given reference frame. The parameter `reference_frame` supports late binding through passing a function. The constructor requires the scene configuration model if a custom reference_frame is used.

```
class PrintPosesInFrame(PrintPose):
    def __init__(self, arm_commander:GeneralCommander=None, scene:Scene=None, reference_frame:str=None):
    ...
```

## Scene manipulation use for simulation

The following behaviour classes can modify the robot configuration by attaching or detaching objects to the robot arm. It is useful for the simulation of grabbing and releasing an object.

### Class task_trees.behavious_base.SimAttachObject

This class attachs an named object to the end-effector link of the robot. 

```
class SimAttachObject(ConditionalCommanderBehaviour):
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                 object_name=None, post_fn=None):
    ...
```
##### extends task_trees.behavious_base.ConditionalCommanderBehaviour

The parameter `post_fn` allows the definition of a function that is called after the object is attached to the robot.

### Class task_trees.behavious_base.SimDetachObject

This class detach an named object from the end-effector link of the robot. 

```
class SimDetachObject(ConditionalCommanderBehaviour):
    def __init__(self, name, condition_fn=True, condition_policy=None, arm_commander=None, 
                 object_name=None, post_fn=None, to_remove=False):
    ...
```
##### extends task_trees.behavious_base.ConditionalCommanderBehaviour

If the parameter `to_remove` is `True`, the detached object is also removed from the world. This feature is useful in simulation.

## Internal use for task management

The classes `HandleTaskCancelled` and `HandleTaskFailure` are used in the skeleton behaviour tree in the task trees manager. These classes are for internal use only.

## Links

- Go back to [Task Trees SDK: Extensible and Adaptable Behaviour Classes](BEHAVIOURS.md)
- Go back to [README: Overview of the Task Trees SDK](README.md)

## Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Mar 2024