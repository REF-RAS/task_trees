# Demo Programs for the Illustration of Task Trees

This page provides an overview of the demo programs for illustration of the **Task Trees SDK**.

### The Major Demo Programs

The following table summarizes the features of each major demo programs or program suites in the `/demo` folder.

| Folder | Name | Application Programming Model | Remarks |
| --- | --- | ---- | ---- |
| gridscan | Pattern Scan in a Grid | Standalone, procedural, non-interactive | |
| pickndrop | Pick N Drop | Standalone, state transition machine | Repeat indefintely |
| pickndrop_estop | Pick N Drop with Estop | Standalone, state transition machine | Pressing EStop aborting all behaviours and the task, EStop as a separate ROS node |
| pushblock | Push a Block between Channels | Standalone, loop with interactive input | Repeat indefintely |
| pushblock_ros | Push a Block between Channels with ROS Client | ROS Server-Client, loop with interactive input in the client | Using ROS Action Goal |

### The Programs for Programming Tutorials

The following demo programs based on only the pre-defined behaviour classes of the task tree SDK and the py_trees Python behaviour trees module.

| Folder | Name | Application Programming Model | Remarks |
| --- | --- | ---- | ---- |
| pytrees_move | Simple movements | Standalone, procedural, repeat indefinitely | Based on PyTrees |
| pytrees_move_scene | Simple movements with logical scene definitions | Standalone, procedural, repeat indefinitely | Based on PyTrees |

The following demo programs based on the task tree manager class `TaskTreesManager` and pre-defined behaviour classes of the task tree SDK.

| Folder | Name | Application Programming Model | Remarks |
| --- | --- | ---- | ---- |
| simple_move | Simple movements | Standalone, procedural, repeat indefinitely | Based on `TaskTreesManager` |
| simple_move_scene | Simple movements with logical scene definitions | Standalone, procedural, repeat indefinitely | Based on `TaskTreesManager` |


## Brief Introduction to the Major Demo Programs

### Pattern Scan in a Grid (gridscan)

The gridscan application simulates the operation of an end-effector working over a grid surface inside a shallow tank. The end-effector moves between grid cells inside the tank according to a program and moves back to a home pose. During the operation, the end-effector changes the orientation while moving to different parts ot the tank.

Go to [README of the Demo Program](../demo/gridscan/DEMO_GRIDSCAN.md)

![The Demo](../demo/gridscan/docs/DemoGridScan1.gif)


### Pick N Drop (pickndrop)

The pick-n-drop application simulates the operation of discovery of a sphere on a desktop, picking it up to disposing it to a bin. The operation will run indefinitely as the simulator creates new spheres at random locations.

Go to [README of the Demo Program](../demo/pickndrop/DEMO_GRIDSCAN.md)

![The Demo](../demo/pickndrop/docs/DemoPickNDrop1.gif)

### Pick N Drop with Estop (pickndrop_estop)

This demo program is an extension of the pick-n-drop application that simulates the operation of discovery of a sphere on a desktop, picking it up to disposing it to a bin. It shows how to use the `GuardedTaskTreesManager` to implement a guard triggered by an EStop button.

Go to [README of the Demo Program](pickndrop_estop/DEMO_PICKNDROP_ESTOP.md)

![The Demo](pickndrop_estop/docs/DemoEStopPickNDrop1.gif)

### Push a Block between Channels (pushblock)

The push-block application simulates a block being moved between 4 side channels involving the end-effector moving into the cavities. The application operates interactively, receiving the target side channel as the destination of the block. The interactive mode can be replaced with the destination randomly drawn.

Go to [README of the Demo Program](pushblock/DEMO_PUSHBLOCK.md)

![The Demo](pushblock/docs/DemoPushBlock1.gif)

### Push a Block between Channels with ROS Client (pushblock_ros)

This demo program is an extension of the push-block application that simulates a block being moved between 4 side channels involving the end-effector moving into the cavities. The application is now restructured as a client-server ROS design. 

Go to [README of the Demo Program](pushblock_ros/DEMO_PUSHBLOCK.md)

![The Demo](pushblock/docs/DemoPushBlock1.gif)


## Programming Tutorials

### Programming Behaviour Tree with purely Py-Trees and the Extension Behaviour Classes

The following two programming tutorials cover the use of extension behaviour classes purely in py-trees. The task trees framework is not used. 

- [Tutorial: Simple Behaviour Tree Programming for Robot Arm Movement](pytrees_moves/TUT_MOVE_PYTREES.md)
- [Tutorial: Exploring Scene Configuration and Custom Reference Frames in the Extension Behaviour Classes](pytrees_moves_scene/TUT_MOVE_SCENE_PYTREES.md)

### Programming Behaviour Tree with the Task Tree Framework and the Extension Behaviour Classes

The following tutorials cover both the extension behaviour classes and the task tree building and management framework.

- [Tutorial: Building Simple Behaviour Trees with the Task Tree Framework and the Extension Behaviour Classes ](simple_moves/TUT_MOVE_TASKTREES.md)
- [Tutorial: Programming Behaviour Trees with Scene Configuration and Custom Reference Frames](simple_moves_scene/TUT_MOVE_SCENE_TASKTREES.md)
- [Tutorial: Developing Advanced and Reusable Behaviour Subtrees with Tasks and the Task Tree Framework](task_moves/TUT_TASK_MOVE_TASKTREES.md)

_Demo Animation of the Task Move 4 Example_ 

![Demo of Task Move 4](task_moves/docs/DemoTaskMove4.gif)



