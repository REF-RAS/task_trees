# The Task Trees Framework

![QUT REF Collection](https://badgen.net/badge/collections/QUT%20REF-RAS?icon=github) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

**Robotics and Autonomous Systems Group, Research Engineering Facility, Research Infrastructure**
**Queensland University of Technology**

## Introduction

The **Task Trees Framework** is a Python programming module for accelerating the development of behaviour tree based robot manipulator applications. It uses the **Arm Commander** [Github Repo](https://github.com/REF-RAS/arm_commander) as the interface to command the robot manipulator. 

![Push-Block Animation](./demos/pushblock/docs/DemoPushBlock1.gif)

Use the [Documentation Entry Point](http://REF-RAS.github.io/task_trees) to bring you to following parts of the documentation of the task trees framework.
- Overview of the Task Trees Framework
- Installation Guide
- The Instant-Use Behaviours
- Logical Scene Configuration Support
- The Task Trees Managers
- Demonstration Programs
- Programming Guide to Instant-Use Behaviours in PyTrees 
- Programming Guide to Instant-Use Behaviours in Task Trees
- Programming Guide to Task Specification and Execution
- API Reference

## Programming with the Task Trees Framework

The task trees framework offers a rich set of concrete instant-use behaviours, behaviour tree templates and application programming interfaces that enables a developer to start experimenting and crafting robotic tasks in a shortened development cycle.

The above demonstration program simulates a robot arm moving a cubic block between four narrow side channels on a surface. Moving the block from the current channel to one of the three other channels can modelled as one task using the task trees framework. Note that there are totally six start-end combinations and also the possibility of the block left in the centre. The task is a sequence of instant-use move and utility behaviours provided by the framework, as shown in the code snippet below.

```
   def create_push_block_task_branch(self) -> Composite:
        # - the branch that executes the task ScanTask
        move_area_branch = py_trees.composites.Sequence('scan_branch', memory=True,
                children=[
                    DoMoveNamedPose('move_home_first_if_inner', ...),
                    DoMoveXYZ('move_to_area_edge', ...), 
                    DoRotate('align_with_the_alley', ...),
                    DoMoveXYZ('move_down', ...),  
                    SimAttachObject('engage_object', ...),    
                    DoMoveXYZ('push_object_to_area_5', ...),    
                    SimDetachObject('detach_at_centre', ...), 
                    DoMoveXYZ('move_up', ...),
                    DoMoveXYZRPY('move_align_for_restart', ...),                    
                    DoMoveXYZ('move_down', ...),                     
                    SimAttachObject('touch_object', ...),
                    DoMoveXYZ('push_object_to_target', ...), 
                    SimDetachObject('detach_object', ...),          
                    DoMoveXYZ('move_up', ...),                   
                    ],
            )
```
There are 9 move behaviours for moving the robotic manipulator and 4 utility behaviours for simulating the block relocation by pushing. The utility behaviours would not be needed in a real environment. No new behaviour class is required in the implementation. 

Refer to the [Design Notes on the PushBlock Demo](docs/source/DEMO_PUSHBLOCK.md) for more details.


## Developer

Dr Andrew Lui, Senior Research Engineer <br />
Robotics and Autonomous Systems, Research Engineering Facility <br />
Research Infrastructure <br />
Queensland University of Technology <br />

Latest update: Mar 2024
