# Introduction to the Instant-Use Behaviour Classes 

The instant-use behaviour (IUB) classes of the task trees framework are all compatible with the [py-trees](https://github.com/splintered-reality/py_trees) module. They share the common base class `py_trees.behaviour.Behaviour`

The majority of the IUB classes supports moving the robotic manipulator. For example, the `DoMoveXYZ` behaviour can move to position specified by xyz subject to a prescribed condition. The rest are some utility behaviours for debugging and simulation, such as the `PrintPose` behaviour can display the pose of the end effector or other links. 

The IUB classes are extensible as new and specialized functionality can be based on the functionality an existing base class. There are generic abstract base classes from which new functonality can be developed. For example, the `ConditionalCommanderBehaviour` has built-in support for managing the commands given to the arm commander, significantly simplifies the development of new move behaviour classes. Another possibility is to extend an existing concrete move behaviour class. For example, the `DoMoveXYZGuardROS` class has inherited the move behaviour from `DoMoveXYZ` and enhance it with sensor-based collision avoidance functionality.  

The [Programming Guide to the Instant-Use Move Behaviour Classes](BEHAVIOURS_GUIDE_MOVE.md) explains using the move behaviours in behaviour tree development.

The [Programming Guide to the Instant-Use Utility Behaviour Classes](BEHAVIOURS_GUIDE_UTILITY.md) explains using the utility behaviours in behaviour tree development.

The [Instruction on Creating New Behaviour Classes](BEHAVIOURS_CREATE.md) describes the hierarchy of base IUB classes and shows how to create new behaviour classes.

The following figure describes all the behaviour classes and their dependencies in the extension package.
![Hierarchy of Behaviour Classes](../assets/TaskTreeBehaviourHierarchy.png)

## PyTree Programming with the IUB Classes

The nominal way is to create instances of IUB classes in a `Composite` branch of a py-tree. The following code snippet comes from `simple_move_1.py` ([source code](../../demos/pytrees_moves/simple_move_1.py)) in the `pytrees_moves` demo folder. It demonstrates adding a behaviour instance of `DoMoveXYZ` as child of a py-tree `Sequence`.

```
class SimpleMovePyTreesApplication():
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        self.arm_commander:GeneralCommander = arm_commander
        self.arm_commander.reset_world()
        self.arm_commander.wait_for_ready_to_move()

        self.root_sequence = self.create_move_branch()
        self.bt = BehaviourTree(self.root_sequence) 

        self.the_thread = threading.Thread(target=lambda: self.bt.tick_tock(period_ms=spin_period_ms), daemon=True)
        self.the_thread.start()  
    
    def create_move_branch(self) -> Composite:
        move_branch = Sequence ('move_branch', memory=True, children=[
                    DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=[0.3, 0.0, 0.6]), ],)
        return move_branch
```
Note that `DoMoveXYZ` is based on `ConditionalCommanderBehaviour` which uses the [arm commander](https://github.com/REF-RAS/arm_commander) for actually executing the move command. The `arm_commander` object, which is associated with a particular move group of the robot manipulator, is an essential parameter. 

The parameter `target_xyz` specifies the target position. In the above example, a constant position is passed to the constructor. `DoMoveXYZ` and most other move IUB classes also accept a string typed logical position (mapping to the physical position according to a scene configuration file) and a function that returns a physical position. The move IUB classes also supports **static and dynamic composition** of the move target, and the use of a **conditional function** to regulate the behaviour execution. Refer to the Programming Guide for the details. 

### Using the IUB Classes with the Task Trees Framework

The IUB classes are best working with the [Task Trees Manager](TASK_TREES_MANAGER.md), which can accelerate the development of more sophisicated behaviour trees. 

The following code snippet comes from `simple_move_1.py` in the `simple_moves` demo folder. The class `TaskTreesManager` offers a behaviour tree template and built-in support for task-to-motion modelling and task execution management. 
```
class SimpleMoveTaskManager(TaskTreesManager):
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        super(SimpleMoveTaskManager, self).__init__(arm_commander)
        ...
        self._add_priority_branch(self.create_move_branch())
        self._install_bt_and_spin(self.bt, spin_period_ms)
    
    def create_move_branch(self) -> Composite:
        move_branch = Sequence('move_branch', memory=True,
                children=[DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=[0.3, 0.0, 0.2]), 
                          DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=[0.3, 0.0, 0.6]),],
        )
        return move_branch
```
The above example uses the template of `TaskTreesManager` for building a behaviour tree. It has not utilized the task execution service of the manager. 

To understand more about the task trees manager, refer to the [Programming Guide to Task Trees Manager](TASK_TREES_MANAGER.md) for instructions and examples.


## Further Information on Programming with the IUB Classes

- [Programming Guide to the Instant-Use Move Behaviour Classes](BEHAVIOURS_GUIDE_MOVE.md)
- [Programming Guide to the Instant-Use Utility Behaviour Classes](BEHAVIOURS_GUIDE_UTILITY.md)
- [Instruction on Creating New Behaviour Classes](BEHAVIOURS_CREATE.md) 
- [Programming Guide to the Task Trees Manager](TASK_TREES_MANAGER.md)

### Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Mar 2024
