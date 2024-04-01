# Programming Guide to Move Behaviours

The programming guide to the instant-use move behaviour classes is covered in the first part. The second part covers the utility instant-use behaviours.

## The Move Target of Instant-Use Move Behaviours 

The move IUB classes are designed to model a robotic manipulator (or one of its parts) moving to a target. As there are different possible forms of move targets, Move IUB classes operating on different move target forms are available for the convenience of developers. The acceptable forms of move targets and their corresponding move IUB class are listed below.
- A logical named pose of joint values: `DoMoveNamedPose`
- A list of physical joint values: `DoMoveJointPose`
- A list of 6 numbers representing the pose in (xyzrpy) format: `DoMovePose`, `DoMoveXYZRPY`
- A list of 7 numbers representing the pose in (xyzqqqq) format: `DoMovePose`
- A `PoseStamped` or `Pose` object: `DoMovePose`
- A list of 3 numbers representing an orientation (rotation) in (rpy) format: `DoRotate`
- A list of 3 numbers representing a position in (xyz) format: `DoMoveXYZ`
- A list of 3 numbers representing a displacement in (dxdydz) format: `DoMoveDisplaceXYZ`
- Multiple poses in a list, of which each pose can be specified as a list of 6 numbers, 7 numbers, `PoseStamped` object or `Pose` object: `DoMoveMultiPoses`
- Multiple positions in a list, of which each position is a list of 3 numbers in (xyz) format: `DoMoveMultiXYZ`

The move target of most of these classes also supports logical binding and dynamic binding, in addition to the usual physical binding. In logical binding, a named move target is decoupled from the actual numbers in the physical scene, and it depends on mapping provided in a scene configuration file. In dynamic binding, the physical move target is queried through a function call at behaviour execution time (i.e. tick-tock time).

A multiple-component move target, such as the xyz or rpy formats, can also be composed of a list of partial specification. For example, the list [[0.2, None, None], [None, 0.6, 0.4]], in a compositional move target, is evaluated to [0.2, 0.6, 0.4]. Note that each partial specification could use logical or dynamic binding, resulting in a very expressive form of specification. 

The move IUB classes support custom frames of reference. Therefore, the move target can be associated with a custom reference frame instead of the __world's frame of reference__. 

## A Summary of Move IUB Classes

**Module: task_trees.behaviours_move**

| Class Name | Target Pose | Target Physical Binding | Target Logical Binding | Target Dynamic Binding | Composition | Reference Frame |
| --- | --- | --- | --- | --- | --- | --- |
| DoMoveNamedPose | `named_pose` | No | Yes | No | No | No |
| DoMoveJointPose | `target_joint_pose` | Yes | No | Yes | No | No | 
| DoMovePose | `target_pose` a list of 6 or 7 numbers, Pose or PoseStamped | Yes | No | Yes | No | Constant and Late Binding |
| DoMoveXYZ | `target_xyz` a xyz list | Yes | Yes | Yes | Yes | Yes |
| DoMoveXYZRPY | `target_xyz` a xyz list and `target_rpy` a rpy list | Yes | Yes | Yes | Yes | Constant and Late Binding |
| DoRotate | `target_rpy` a rpy list | Yes | Yes | Yes | Yes | Constant and Late Binding |
| DoMoveDisplaceXYZ | `dxyz` a list representing xyz displacement | Yes | Yes | Yes | Yes | Constant and Late Binding |
| DoMoveMultiPoses | `target_poses` a list of poses as in `DoMovePose`| Yes | No | Yes | No | Constant and Late Binding |
| DoMoveMultiXYZ | `target_xyz_list` a list of xyz | Yes | Yes | Yes | Yes | Constant and Late Binding |

**Module: task_trees.behaviours_move_sense**

The following class is a specialization of `DoMoveXYZ`, providing abortion of motion based on sensor-based collision detection. 

| Class Name | Target Pose | Constant Binding | Logical Binding | Late Binding | Composition | Reference Frame |
| --- | --- | --- | --- | --- | --- | --- |
| DoMoveXYZGuardROS | `target_xyz` a xyz list | Yes | Yes | Yes | Yes | Yes |

All the above move behaviour classes are based on the abstract classes of `ConditionalBehaviour` and `ConditionalCommanderBehaviour`. The first base class provides the support of conditional execution of behaviours and the second base class manages the execution of move commands. Most of the classes are also based on the abstract class of `SceneConditionalCommanderBehaviour`, which supports using logical scene poses in the specification of move targets. 

The parentage of the motion behaviour classes is depicted below. 

![Hierarchy of Move Behaviour Classes](../assets/HierarchyMoveClasses.png)


## Using Different Bindings on Move Targets

The effective binding mode to use on a move target depends on the type. For example, the class `DoMoveXYZ` treats the parameter `target_xyz` according to whether it is a list of numbers, a string, a function, or a list consisting elements of the prior three types. 
- **A list of 3 constant numbers** is considered as a physical position specified in xyz (i.e. physical binding).
- **A string** is considered as a logical position specified in the scene configuration file (i.e. logical binding). If the logical position is not found in the configuration file, an exception is raised. 
- **A reference to a function definition** is considered as a source where the physical position can be dynamically queried by a function call (i.e. dynamic binding).
- **A list consisting elements of the above three types** is considered a compositional move target. Each element is then resolved to a physical xyz individually according to its type, and will then be recomposed into a xyz position.


### Physical (Scene) Binding 

A constant move target is etched in the program and its value is finalized and cannot be changed during execution (tick-tock) time. It requires no further resolution. 

A drawback of using physcial binding is the tight coupling to a particular physical scene. A change to the physical scene or the robot manipulator necessities a change of the program code.

```
    move_branch = py_trees.composites.Sequence('move_branch', memory=True,
            children=[
                DoMoveXYZ('move_start', True, arm_commander=self.arm_commander, target_xyz=[0.2, 0.3, 0.5]), 
                ],)
```

In the above example, if a robot arm model of different physcial attributes needs a change in the source code.

### Logical (Scene) Binding 

A string representation of a move target is resolved during the execution (tick-tock) time, based on a mapping provided in the scene configuration file. 

The following shows an example of using logical position and rotation names as values for the parameter `target_xyz`.
```
    move_branch = py_trees.composites.Sequence(
            'move_branch',
            memory=True,
            children=[
                DoMoveXYZ('move_start', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='positions.start'), 
                DoMoveXYZ('move_end', True, arm_commander=self.arm_commander, scene=self.the_scene, target_xyz='positions.end'), 
                ],)
```
The parameter `scene` specify the scene configuration that defines the logical names. The following is the statement for loading the scene configuration from the file `task_scene_2.yaml`.
```
    self.the_scene = Scene(os.path.join(os.path.dirname(__file__), 'task_scene_2.yaml'))
```
The content of the file is shown below.
```
scene:
  named_poses: 
    stow: [0.0, -1.244, 0.0, -2.949, 0.0, 1.704, 0.785] # from base
    home: [0.0, -0.785, 0.0, -2.36, 0.0, 1.57, 0.785] # from base
  positions:
    start: [0.3, 0.0, 0.2]
    end: [0.3, 0.0, 0.6]
subscenes:
...
```
An advantage of using logical positions instead of physical positions is the decoupling from the actual physical scene. A change to the physical scene or the robot model only necessities a change in the configuration file or replacing one configuration file with another. No change in the source code is required. 

### Dynamic Binding

Passing a function reference as a move target will have the target resolved at execution (tick-tock) time by a function call. The function is expected to return the required move target in physical form. 

In the example below, the sequence of behaviours aims to grab a target object and drop it at a random position. The position of the target object has to be determined on the fly. The function `get_object_position` uses a sensor to estimate the best grab position of the target. The function reference is passed as the parameter `target_xyz`. It will get resolved everytime the behaviour is executed. The binding of the parameter is delayed until tick-tick time.  Similarly, the random position should change every time and therefore the function `generate_random_xyz` is passed as the move target for the second `DoMoveXYZ` behaviour instance.

```
def get_object_position(self) -> list:
    # obtain the best object position for the gripper from a sensor
    ...
    return xyz

def generate_random_xyz(self) -> list:
    xyz = [random.uniform(0.1, 0.5), random.uniform(-0.3, 0.3), random.uniform(0.2, 0.6)]
    return xyz
...
def build_tree(self) -> Composite:
...
    move_branch = py_trees.composites.Sequence( 'move_branch', memory=True,
            children=[
                DoMoveXYZ('move_to_object', True, arm_commander=self.arm_commander, target_xyz=self.get_object_position),
                CloseGripper('grab_the_object', True, arm_commander=self.arm_commander),
                DoMoveXYZ('move_random_xyz', True, arm_commander=self.arm_commander, target_xyz=self.generate_random_xyz),
                OpenGripper('drop the object', True, arm_commander=self.arm_commander),
                ],)
```

The power of dynamic binding is in on-the-fly generation of move targets. The above example


### Composition of the Move Target

A compositional move target specification builds the parameter value from a list of pose items. 

The following example shows that the final xyz is composed from a logical position `positions.xy_start` that defines the x and y, and another logical position `positions.z_upper` that defines z. 
```
    DoMoveXYZ('move_start', True, arm_commander=self.arm_commander, scene=self.the_scene, 
        target_xyz=['positions.xy_start', 'positions.z_upper']), 
```
The definitions of the two logical position names are given below. 
```
scene:
  positions:
    xy_start: [0.2, 0.3, null]
    xy_mid: [0.25, 0.4, null]
    xy_end: [0.3, 0.6, null]
    z_upper: [null, null, 0.4]
    z_lower: [null, null, 0.2]
  rotations:
    alpha: [3.139, 0.0, -0.785]
subscenes:
```
The compositional list is processed from the front to the end or until all components are specified. The first item `positions.xy_start` has omitted the z component and therefore the second item is examined for the missing component. The final `target_xyz` is computed as `[0.2, 0.3, 0.4]`

Compositional specification enables the separation and remixing of positional and rotational components. The following figure illustrates how 7 partial position definitions can define 12 positions without redundency, making the specification resiience to change.

![Compositional Position](../assets/TutorialCompositionTarget.png)


### Moving in a Custom Reference Frame

The parameter `reference_frame` accepts both the frame name representing a constant string and a function that returns a constant string. The following examples shows two `DoMoveXYZ` behaviours moving to different physical positions, though the target position is the same [0, 0, 0].  The frame `area_1` and `area_2` respectively determine the binding to the physical positions. 
```
DoMoveXYZ('move_origin_area_1', True, arm_commander=self.arm_commander, target_xyz=[0, 0, 0], reference_frame='area_1'),
DoMoveXYZ('move_origin_area_1', True, arm_commander=self.arm_commander, target_xyz=[0, 0, 0], reference_frame='area_2')
```

The parameter `reference_frame` supports late binding as well. If the parameter value is a function definition, the reference frame is obtained when the move behaviour is actually executed.

#### Examples

For more examples on compositional move targets and logical position and rotation names, refer to the following tutorial pages.
- [Tutorial: Instant-Use Move Behaviours in PyTrees](TUT_MOVE_PYTREES.md)
- [Tutorial: Instant-Use Move Behaviours in PyTrees using Logical Scenes](TUT_MOVE_SCENE_PYTREES.md)


### Conditional Behaviours

The move behaviour classes are conditional behaviours, meaning that a condition function can be optionally passed as a parameter to the constructor. When the tick happens at the behaviour in a Sequence, a False returned from the condition function will considere the behaviour SUCCESS and the tick is moved to the next behaviour. This means the outcome of this behaviour is considered achieved and executing the behaviour is not needed. On the other hand, True condition will result in the execution of the behaviour. 

Building a behaviour subtree based on conditional behaviours can adapt a sequence of behaviours to different starting state of the robot arm. Because it enables selectively execute a list of behaviours in a Sequence (and Selector), some conditional behaviours can be added for different starting states moving to planned intermediate states.

Conditional behaviour classes have separated conditions from actions in behaviours. The following example shows a sequence of behaviours designed for pick-n-drop. The ovals are the actions to be taken if the associated condition, in a rectangle, is True. The second behaviour of which the action is _move down_ is required only if the gripper is located high up, which is specified as the condition. Examine the other conditional behaviours in the sequence and the conditions basically represent the inverse of the desirable outcome of the behaviours.  

#![Conditional Behaviour Example](../assets/ConditionalBehavioursInTree.png)

#### Convergence Pattern Resulting from Conditional Behaviours

Conditional behaviour classes are pivotal in the implementation of the following convergence pattern of behaviour subtrees. A use case of this pattern is to bring different starting poses through appropriate intermediate poses and finally to converge into the same outcome. 

#![Convergence Pattern](../assets/ConditionalBehaviourConvergence.png)

#### Acceptable Forms of Condition Functions

The parameter `condition_fn` accepts many forms, which are described in the below table. 

#![Conditional Function Spec](../assets/ConditionalFunctionSpec.png)

The actual definitions of condition functions are usually defined in the task trees manager (i.e. a subclass) so that these functions have access to the other instance variables such as the `arm_commander`.

#### Example Behaviour Specifications

The parameter `condition_fn` in the constructor of subclasses of `ConditionalBehaviour` is queried at the ticking time of the behaviour. If the parameter is True, either from a constant or one or more conditional functions, the behaviour is executed. The behaviour is _skipped_ if otherwise, and in this case the next behaviour (i.e. the next sibling) under the same composite parent is ticked. 

Consider the following example exerpted from the GridScan demo program. The ellipsis indicates omitted parameters `arm_commander` and `scene` for clarity. 
```
py_trees.composites.Sequence('move_named_pose_branch', memory=True,
                children=[
                    DoMoveXYZ('move_up_if_in_water', condition_fn= [{'_fn': self.in_a_region, 'region': 'regions.work'}, 
                                                                    {'_fn': self.below_z, 'position':'tank.positions.hover'}], 
                                    ..., target_xyz='tank.positions.hover', reference_frame='the_tank' cartesian=True),                    
                    DoMoveNamedPose('move_home_first', condition_fn=[{'_fn': self.in_a_region, 'region': 'regions.work'}, 
                                                                    {'_fn': self.on_or_above_z, 'position':'tank.positions.hover'}], 
                                    ..., named_pose='named_poses.home'),
                    DoMoveNamedPose('move_task_pose', condition_fn=True, ..., named_pose=self.query_grid_position_of_task),                            
                    ],
            ),
```
Notes:
- The first behaviour named `move_up_if_in_water` is executed if both functions (`self.in_a_region` and `self.below_z`) in the `condition_fn` returns True. The first function returns True if the end-effector is in a logical region `regions.work`. The second function returns True if the z position of the end effector is higher than the logical z position `tank.positions.hover`.
- If the condition of the first behaviour is False, the behavour is skipped. The second behaviour is then ticked. 
- The condition mechanism does not affect the prescribed logic of `Sequence` or `Selector`. It only affects whether individual behaviour is actually executed at ticking.  

### Multi-Pose Move Behaviour Classes

The behaviour classes `DoMoveMultiXYZ` and `DoMoveMultiPose` supports continuous movement made up of multiple waypoints or intermediate poses. The use cases are quite different.

The behaviour class `DoMoveMultiXYZ` supports compositional waypoint xyz positions, and physical, logical, and late binding of every waypoint positions. The rotation will remain unchanged at each waypoint. Specification of the waypoints is visible in the behaviour tree building code.

The other multi-pose behaviour class `DoMoveMultiPose` supports specification of both position and rotation in every waypoint pose, but does not support compositional nor logical and late binding of every waypoint pose. It does support tick-tock time generation of the list of waypoints by a function, however, the waypoints are no longer visible in the behaviour tree building code.

The following shows an example of defining a multiple pose motion, exerpted from `multi_move_1.py` [source code](../../demos/simple_moves/multi_move_1.py) in the folder `demos/simple_moves`.
```
    def create_move_branch(self) -> Composite:
        move_branch = Sequence('move_branch', memory=True,
                children=[
                    DoMoveMultiPose('move_xyz', True, arm_commander=self.arm_commander, target_poses=[
                        (0.6, 0.0, 0.4, 3.14, 0, 0), 
                        (0.6, 0.2, 0.5, 3.14, 0, 0), 
                        (0.6, 0.2, 0.6, 3.14, 0, 1.58), 
                        (0.6, 0.0, 0.7, 3.14, 0, 3.14), 
                        (0.6, -0.2, 0.6, 3.14, 0, 1.58), 
                        (0.6, -0.2, 0.5, 3.14, 0, 0), 
                        (0.6, 0.0, 0.4, 3.14, 0, 0),
                        ]), 
                    ],
        )
        return move_branch
```

![Multi Move 1](../../demos/simple_moves/docs/TutorialMultiMove1.gif)

### Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Mar 2024
