# Task Trees SDK: Extensible and Adaptable Behaviour Classes

The [py-trees](https://github.com/splintered-reality/py_trees) behaviour class extension package of the **Task Trees SDK** aims to ease the implementation of behaviour tree based robot arm manipulation. The behaviour classes in the package are based on class `py_trees.behaviour.Behaviour`, enabling seamless slot into a py-tree behaviour tree. The behaviour classes can be divided into three groups.

- **Move Behaviours**. The move behaviour classes can instruct a robot arm to execute various primitive movements. These classes have built-in connection to the [moveit](https://ros-planning.github.io/moveit_tutorials/) platform through the [arm commander](https://github.com/REF-RAS/arm_commander). Developers can exploit these existing classes to populate their behaviour trees.
- **Extensible Base Behaviours**. The extensible base behaviour classes comprise of built-in logic that deals with interaction with the underlying arm commander, and other typical components in a robot arm manipulation applications. Developers can use these functionally rich base classes to design custom behaviour classes.
- **Utility Behaviours**. The utility behaviour classes include those that support debugging of bahaviour trees and building a simulation application.  

The following figure describes all the behaviour classes and their dependencies in the extension package.
![Hierarchy of Behaviour Classes](docs/assets/TaskTreeBehaviourHierarchy.png)

The extension package supports further specialization on the existing classes. For example, the move behaviour classes can form the basis for custom sensor-integration classes such as `DoMoveXYZGuardROS`.

## Programming with the Behaviour Classes

Simply slot one of the movement behaviour classes, for example the `DoMoveXYZ`, into a py-tree in a program to implement behaviour tree controlled robot arm. The following code snippet comes from `simple_move_1.py` in the `pytrees_moves` demo folder. 
```
class SimpleMovePyTreesApplication():
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        signal.signal(signal.SIGINT, self.stop)
        self.arm_commander:GeneralCommander = arm_commander
        ...
        # build the behaviour tree
        self.root_sequence = self.create_move_branch()
        self.bt = BehaviourTree(self.root_sequence) 
        self.the_thread = threading.Thread(target=lambda: self.bt.tick_tock(period_ms=spin_period_ms), daemon=True)
        self.the_thread.start() 
    
    def create_move_branch(self) -> Composite:
        move_branch = Sequence (
                'move_branch',
                memory=True,
                children=[DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=[0.3, 0.0, 0.6]),],
        )
        return move_branch
```
Refer to [Tutorial: Simple Behaviour Tree Programming for Robot Arm Movement](pytrees_moves/TUT_MOVE_PYTREES.md) for more examples.

### Using the Behaviour Classes with the Task Trees Framework

Instead of using the py-trees, the [Task Trees framework](TASK_TREES_MANAGER.md) offers functionality that enables easier implementation of more sophisicated behaviour trees. 

The following code snippet comes from `simple_move_1.py` in the `simple_moves` demo folder. The class `TaskTreesManager` offers a pre-built skeleton behaviour tree and the logic for its operation. 
```
class SimpleMoveTaskManager(TaskTreesManager):
    def __init__(self, arm_commander:GeneralCommander, spin_period_ms:int=10):
        super(SimpleMoveTaskManager, self).__init__(arm_commander)
        ...
        # build and install the behavior tree
        self._add_priority_branch(self.create_move_branch())
        self._install_bt_and_spin(self.bt, spin_period_ms)
    
    def create_move_branch(self) -> Composite:
        move_branch = Sequence (
                'move_branch',
                memory=True,
                children=[DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=[0.3, 0.0, 0.2]), 
                          DoMoveXYZ('move_xyz', True, arm_commander=self.arm_commander, target_xyz=[0.3, 0.0, 0.6]), ],
        )
        return move_branch
```
Refer to [Tutorial: Building Simple Behaviour Trees with the Task Tree Framework and the Extension Behaviour Classes ](simple_moves/TUT_MOVE_TASKTREES.md) for more examples.

The Task Trees Manager allows the specification of tasks, each of which is a logical sequence of behaviours. For details, please refer to the page on [Task Trees framework](TASK_TREES_MANAGER.md).

## Links

The descriptions of the three groups of behaviour classes can be found in their respective pages.

- [The Move Behaviours](BEHAVIOURS_MOVE.md)
- [The Base Behaviours](BEHAVIOURS_BASE.md)
- [The Utility Behaviours](BEHAVIOURS_UTILITY.md)

You can also go back to [README: Overview of the Task Trees SDK](README.md)

## Author

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Mar 2024
