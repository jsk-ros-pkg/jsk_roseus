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

To do this we add an action with the `topic_name` and `to` fields in the `<BehaviorTree/>` section and declare it as a `<Subscriber/>` and specify its type in the `<TreeNodesModel/>` section.
Only proper ROS message types are supported by subscriber nodes (e.g. `std_msgs/Int64` instead of `int64`).

Note how we also add a step to verify and wait for messages.


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

Because in such scenario the condition nodes must be evaluated alongside the running action we prepare two distinct roseus servers -- one for actions and one for conditions.
On the action side it is also necessary to check for the presence of interruption requests with the `(roseus_bt:ok)` function.

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

The final example https://github.com/Affonso-Gui/jsk_roseus/blob/roseus_bt/roseus_bt/sample/models/t07_xacro.xml.xacro illustrates how we can use the xacro package to improve readability and modularity of the model file descriptions.

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