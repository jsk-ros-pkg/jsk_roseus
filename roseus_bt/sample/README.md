# roseus_bt_tutorials

Create and build the tutorial package

```bash
cd ~/catkin_ws/src
rosrun roseus_bt create_bt_tutorials
catkin build roseus_bt_tutorials
```

## t01_simple_tree
![t01](https://user-images.githubusercontent.com/20625381/125036489-082d3f80-e0ce-11eb-8cce-d87a06b2c1d8.gif)

We start with a simple behavior tree, composed only by a few actions organized into a single sequence.
The model file https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t01_simple_tree.xml is divided into the following two main sections:
- `<BehaviorTree/>` specifies the tree structure
- `<TreeNodesModel/>` specifies the custom node palette

Every `<Action/>` tag in the `<TreeNodesModel/>` must be provided with an arbitrary `server_name` field.

The recommended way to write a xml model file is to use the Groot editor and then edit in the required fields afterwards.

Both the `<BehaviorTree/>` tag in the xml model file and the euslisp server are loaded at runtime, but changes in the node palette (`<TreeNodesModel/>`) must be re-generated and re-compiled.

#### Run the code

Run the roseus server
```bash
roscd roseus_bt_tutorials/euslisp
roseus t01_simple_tree-action-server.l
```

Run the cpp client
```bash
rosrun roseus_bt_tutorials t01_simple_tree
```

Optionally run Groot for visualization
```bash
rosrun groot Groot
```

## t02_conditions
![t02](https://user-images.githubusercontent.com/20625381/125036852-707c2100-e0ce-11eb-99b8-a8d568e6e97c.gif)

The second example https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t02_conditions.xml also includes condition and fallback nodes.

Every `<Condition/>` tag in the `<TreeNodesModel/>` must be provided with an arbitrary `service_name` field.

#### Run the code

Run the roseus server
```bash
roscd roseus_bt_tutorials/euslisp
roseus t02_conditions-action-server.l
```

Run the cpp client
```bash
rosrun roseus_bt_tutorials t02_conditions
```

Optionally run Groot for visualization
```bash
rosrun groot Groot
```

## t03_ports
![t03](https://user-images.githubusercontent.com/20625381/125036607-25faa480-e0ce-11eb-9013-28b2c41c90f2.gif)

The third example https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t03_ports.xml introduces Ports, which act as the input and output arguments for nodes.

Ports are strongly typed and can take any type which can be used in a ROS message field (e.g. `int64` and `int32` are accepted but `int` is not supported).

Port variables can be assigned/referenced with the `${variable_name}` syntax and are stored in the behavior tree blackboard.

The name and type of each node port are specified in the `<TreeNodesModel/>` tag, and its value in the `<BehaviorTree/>` tag.

Ports can be declared as either `<input_port/>`, `<output_port/>` or `<inout_port/>`.
Input ports are passed to the roseus layer as function arguments; Output ports can be set at any point of execution through the `(roseus_bt:set-output "name" value)` function.

Conditions only support input ports, as they are not meant to do any changes in the behavior tree state.


#### Run the code

Run the roseus server
```bash
roscd roseus_bt_tutorials/euslisp
roseus t03_ports-action-server.l
```

Run the cpp client
```bash
rosrun roseus_bt_tutorials t03_ports
```

Optionally run Groot for visualization
```bash
rosrun groot Groot
```

## t04_subscriber
![t04](https://user-images.githubusercontent.com/20625381/125036625-2b57ef00-e0ce-11eb-8198-974d1b45855a.gif)

The fourth example https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t04_subscriber.xml shows how we can remap topic messages to port variables.

Such port variables are initialized with an empty message instance and updated every time a new topic message arrives.

To do this we add a `<Subscriber/>` node, specifying the input ports `topic_name` and `message_type` and the output ports `output_port` and `received_port`. The `output_port` variable is initilized with an instance of the given message type and updated every time a new message is received. The `received_port` variable is a boolean initialized with false and set to true at every new message. Optionally, `message_field` can also be assigned.

Only proper ROS message types are supported by subscriber nodes (e.g. `std_msgs/Int64` instead of `int64`).


#### Run the code

Publish the topic:
```bash
rostopic pub -r 10 /petbottle/coords geometry_msgs/Pose "position:
  x: 1.85
  y: 0.5
  z: 0.7
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0"
```

Run the roseus server
```bash
roscd roseus_bt_tutorials/euslisp
roseus t04_subscriber-action-server.l
```

Run the cpp client
```bash
rosrun roseus_bt_tutorials t04_subscriber
```

Optionally run Groot for visualization
```bash
rosrun groot Groot
```

## t05_subtrees
![t05](https://user-images.githubusercontent.com/20625381/125036658-3448c080-e0ce-11eb-9592-23f6b424bcd1.gif)

The fifth example https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t05_subtrees.xml wraps up the previous task in a subtree and adds another example task.

Nested trees can be defined with multiple `<BehaviorTree/>` tags and referenced with the `<SubTree/>` tag.

Each subtree inherits a separate blackboard and accepts remaps in the `inner_name="outer_name"` syntax.

#### Run the code

Run the roseus server
```bash
roscd roseus_bt_tutorials/euslisp
roseus t05_subtrees-action-server.l
```

Run the cpp client
```bash
rosrun roseus_bt_tutorials t05_subtrees
```

Optionally run Groot for visualization
```bash
rosrun groot Groot
```

## t06_reactive
![t06](https://user-images.githubusercontent.com/20625381/125036676-390d7480-e0ce-11eb-813e-69784c2053a9.gif)

The sixth example https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t06_reactive.xml uses reactive fallbacks to constantly check and respond to user requests.

The main difference of reactive nodes (e.g. `<ReactiveSequence/>` and `<ReactiveFallback/>`) is that when a child returns RUNNING the reactive node will resume ticking from its first child. This forces the node to re-evaluate any conditions preceding the execution node, therefore achieving enhanced reactivity.

In order to compose a reactive program in single-threded eus, we need to ensure that:
1. Condition nodes can be evaluated while executing actions
2. Interruption requests are checked while executing actions

In this example, we achieve the first point by preparing two distinct roseus servers -- one for actions and the other for conditions.
The second point is achieved by constantly spinning the ros node during the execution loop, which by default raises a `roseus_bt:cancel-action` condition whenever an interruption request is received. Finally, the execution callback is also responsible for correctly handling this condition.

When using the real robot, the suggested way to achieve concurrent evaluation is to register condition nodes `:groupname` and action nodes `:monitor-groupname` with the robot's interface groupname (e.g. `(*ri* . groupname)`). This allows for checking conditions and interruption requests at every robot spin, which naturally happens at several points of the control loop, such as during `:wait-interpolation`.

It is also possible to manually spin the monitor groupname with `(send server :spin-monitor)`, or to check for the presence of interruption requests with the `(send server :ok)` when using custom preemption callbacks.

#### Run the code

Run the roseus action server
```bash
roscd roseus_bt_tutorials/euslisp
roseus t06_reactive-action-server.l
```

Run the roseus condition server
```bash
roscd roseus_bt_tutorials/euslisp
roseus t06_reactive-condition-server.l
```

Run the cpp client
```bash
rosrun roseus_bt_tutorials t06_reactive
```

Send the request
```bash
rostopic pub --once /get_drink/request std_msgs/Bool true
```

Optionally run Groot for visualization
```bash
rosrun groot Groot
```

## t07_xacro

In this example https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t07_xacro.xml.xacro we illustrate how it is possible to use the xacro package to improve readability and modularity of the model file descriptions.

The following will create the `t07_xacro.xml` file equivalent to the `t05_subtrees.xml` example
```bash
roscd roseus_bt/sample/models
rosrun xacro xacro t07_xacro.xml.xacro -o t07_xacro.xml
```

And it is also possible to create independent models for each of the subtrees (in this case setting the `main` argument)
```bash
rosrun xacro xacro t07_xacro_pour_task.xml.xacro -o t07_xacro_pour_task.xml main:=true
rosrun xacro xacro t07_xacro_sweep_task.xml.xacro -o t07_xacro_sweep_task.xml main:=true
```

Note how port variables need to be quoted (e.g.`$${var}`) to use the xacro syntax.
The `{var}` notation also works.

## t08_multimaster

This example shows how to use the rosbridge interface to assign different hosts to each action in a multimaster application.
https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t08_multimaster.xml

To do this we declare the actions with the `<RemoteAction/>` and conditions with the `<RemoteCondition/>` tag in the `<TreeNodesModel/>`, and add a `host_name` and `host_port` field to them.

Make sure that the rosbridge server is started after sourcing all of the package's messages and services. Setting a large `unregister_timeout` is also desirable to avoid problems described in https://github.com/knorth55/jsk_robot/pull/230 .


#### Run the code

Run the first rosbridge_server:
```bash
# source package before running this
roslaunch rosbridge_server rosbridge_websocket.launch unregister_timeout:=100000
```

Run the first roseus server:
```bash
roscd roseus_bt_tutorials/euslisp
roseus t08_multimaster_localhost9090-action-server.l
```

Run the second rosbridge_server:
```bash
# source package before running this
export ROS_MASTER_URI=http://localhost:11312
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091 unregister_timeout:=100000
```

Run the second roseus server:
```bash
export ROS_MASTER_URI=http://localhost:11312
roscd roseus_bt_tutorials/euslisp
roseus t08_multimaster_localhost9091-action-server.l
```

Run the cpp client
```bash
rosrun roseus_bt_tutorials t08_multimaster
```

Optionally run Groot for visualization
```bash
rosrun groot Groot
```

