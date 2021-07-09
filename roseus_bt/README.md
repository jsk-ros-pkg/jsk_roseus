roseus_bt
============

Generate glue code for connecting your roseus projects to [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP), [BehaviorTree.ROS](https://github.com/BehaviorTree/BehaviorTree.ROS) and [Groot](https://github.com/BehaviorTree/Groot).

## What are behavior trees

Behavior Trees are a control structure used to better organize and design the logical flow of an application. When compared to State Machines they tend to display increased modularity (because there are no direct connections between the execution nodes) and reactivity (because the tree formulation includes implicit transitions).

BehaviorTree.CPP documentation page: [https://www.behaviortree.dev/bt_basics](https://www.behaviortree.dev/bt_basics)

Behavior Tree in Robotics and AI: [https://arxiv.org/pdf/1709.00084.pdf](https://arxiv.org/pdf/1709.00084.pdf)

## Quick Setup

Clone related directories
```bash
mkdir ~/catkin_ws/src -p
cd ~/catkin_ws/src
wget https://raw.githubusercontent.com/Affonso-Gui/jsk_roseus/roseus_bt/roseus_bt/roseus_bt.rosinstall -O .rosinstall
wstool update
```

Install dependencies
```bash
rosdep install -yr --ignore-src --from-paths BehaviorTree.ROS Groot jsk_roseus/roseus_bt
```

Build & source
```bash
cd ~/catkin_ws/
catkin build roseus_bt groot
source ~/catkin_ws/devel/setup.bash
```

## Run

The following command creates a new ros package with all the necessary glue code for running roseus code on the BehaviorTree.CPP engine.

```bash
cd ~/catkin_ws/src
rosrun roseus_bt create_bt_package my_package path/to/model.xml
catkin build my_package
```

For more information on how to compose the model file and use the package check the [tutorials](https://github.com/Affonso-Gui/jsk_roseus/tree/roseus_bt/roseus_bt/sample) and the [BehaviorTree.CPP documentation](https://www.behaviortree.dev/bt_basics).

## Samples

Follow instructions at the [tutorial page](https://github.com/Affonso-Gui/jsk_roseus/tree/roseus_bt/roseus_bt/sample).
