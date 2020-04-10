roseus_smach
============

This package includes euslisp implementation of state machine and [smach](http://wiki.ros.org/smach).

## requirements

- [roseus](http://wiki.ros.org/roseus)
- [smach](http://wiki.ros.org/smach)
- [smach_viewer](http://wiki.ros.org/smach_viewer)
  - Optional for visualization.
  - You have to install this package manually:\
    `sudo apt install ros-$ROS_DISTRO-smach-viewer`

## sample

Sample codes are available on `sample` directory.

- `rosrun roseus_smach state-machine-ros-sample.l`
  - simple state machine
  ![](http://gist.github.com/furushchev/9b1ed0aa57b47537cd2d/raw/smach-simple.gif)
  ```
  rosrun smach_viewer smach_viewer.py
  ```
  ```lisp
  rosrun roseus_smach state-machine-ros-sample.l
  (exec-smach-simple)
  ```
  - nested state machine
  ![](http://gist.github.com/furushchev/raw/9b1ed0aa57b47537cd2d/smach-nested.gif)
  ```
  rosrun smach_viewer smach_viewer.py
  ```
  ```lisp
  rosrun roseus_smach state-machine-ros-sample.l
  (exec-smach-nested)
  ```
  - state machine with userdata
  ![](http://gist.github.com/furushchev/9b1ed0aa57b47537cd2d/raw/smach-userdata.gif)
  ```
  rosrun smach_viewer smach_viewer.py
  ```
  ```lisp
  rosrun roseus_smach state-machine-ros-sample.l
  (exec-smach-userdata)
  ```

- `sample/parallel-state-machine-sample.l`

  - state machine with parallel action execution
  ![](http://gist.github.com/furushchev/9b1ed0aa57b47537cd2d/raw/smach-parallel.gif)
  ```
  rosrun smach_viewer smach_viewer.py
  ```
  ```lisp
  rosrun roseus_smach parallel-state-machine-sample.l
  (demo)
  ```
## Writing Simple Smach(state-machine)
Example codes are [here](https://github.com/jsk-ros-pkg/jsk_roseus/blob/master/roseus_smach/sample/state-machine-sample.l).
```lisp
#!/usr/bin/env roseus
(load "package://roseus_smach/src/state-machine.l")

(setq count 0)
(defun func-foo (userdata-alist)
  (format t "Execute state FOO~%")
  (cond ((< count 3) (incf count) :outcome1)
	(t :outcome2)))
(defun func-bar (userdata-alist)
  (format t "Execute state BAR~%")
  :outcome2)

(defun smach-simple ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :FOO 'func-foo))
    (send sm :add-node (instance state :init :BAR 'func-bar))
    ;; goal-states are generated in this method
    (send sm :goal-state (list :outcome4 :outcome5))

    ;; select a node as start-node
    (send sm :start-state :FOO)
    ;; from and to nodes are selected by name or symbol
    (send sm :add-transition :FOO :BAR :outcome1)
    (send sm :add-transition :FOO :outcome4 :outcome2)
    (send sm :add-transition :BAR :FOO :outcome2)
    sm ))

(send (smach-simple) :execute nil)
```
### The Code Explained
```lisp
(load "package://roseus_smach/src/state-machine.l")
```
This line imports state-machine class, state class, and transition class.



```lisp
(defun func-foo (userdata-alist)
  (format t "Execute state FOO~%")
  (cond ((< count 3) (incf count) :outcome1)
	(t :outcome2)))
(defun func-bar (userdata-alist)
  (format t "Execute state BAR~%")
  :outcome2)
```
These lines define some functions that will be hooked to state. Note that the functions are called with one argument, an alist of the declared user-data arguments, and that their return value is equal to the transition (or the action) of the state.


```lisp
(sm (instance state-machine :init))
```
This line creates the state-machine instance and binds it to `sm`.

```lisp
(send sm :add-node (instance state :init :FOO 'func-foo))
```
This line creates a new state and add the node to the state machine. When you create new state instance, you can name the state and hook the state to a function. Any lisp object can be used for state names. In this case, a new state has name `:FOO` and hooked with `func-foo`.

```lisp
(send sm :goal-state (list :outcome4 :outcome5))
(send sm :start-state :FOO)
```
These lines define goal state(s) and start state(s) to the state machine. You can create multiple start states as a list of state, also you can define multiple goal state. Goal state(s) do nothing and returns its name.   Note that **a state machine is also a state**. So the goal state can be considered as the outcome of the state machine.


```lisp
(send sm :add-transition :FOO :BAR :outcome1)
 ```
 This line defines transition. The arguments of `:add-transition method` is `TO`, `FROM`, `OUTCOME`. So this line means that "adding transition from FOO to BAR, when FOO node returns outcome1".

```lisp
(send (smach-simple) :execute nil)
```
This line sets up and execute the state machine.
In order to view and check the progress with [smach_viewer](http://wiki.ros.org/smach_viewer), use the function [exec-state-machine](https://github.com/jsk-ros-pkg/jsk_roseus/blob/master/roseus_smach/src/state-machine-utils.l#L3) instead.
## Writing Nested Smach
Example codes are [here](https://github.com/jsk-ros-pkg/jsk_roseus/blob/01320ceaf72857404746171cbaa0f1724e4ad4b8/roseus_smach/sample/state-machine-sample.l#L42).
You can also add child state-machine as a node to state-machine.
![](https://gist.github.com/ykawamura96/987e67b3775d68cac78031b994c3a0ba/raw/865d64ec00d2dfa6f6f1b03735918ac6f65767bc/nested_samch.png)
```lisp
#!/usr/bin/env roseus
(load "package://roseus_smach/src/state-machine.l")
(defun smach-nested ()
  (let ((sm-top (instance state-machine :init))
        (sm-sub (instance state-machine :init)))
    ;; state instance can include other state-machine like function
    (send sm-top :add-node (instance state :init "SUB" sm-sub))
    (send sm-top :add-node (instance state :init "BAS" 'func-bas))
    (send sm-top :goal-state :outcome5)
    (send sm-top :start-state "BAS")
    (send sm-top :add-transition "BAS" "SUB" :outcome3)
    (send sm-top :add-transition "SUB" :outcome5 :outcome4)
    ;; node instance can be args of :add-node, :start-state, :add-transition
    (let ((foo-node (instance state :init "FOO" 'func-foo))
          (bar-node (instance state :init "BAR" 'func-bar)))
      (send sm-sub :add-node foo-node)
      (send sm-sub :add-node bar-node)
      (send sm-sub :goal-state :outcome4)
      (send sm-sub :start-state foo-node)
      (send sm-sub :add-transition foo-node bar-node :outcome1)
      (send sm-sub :add-transition foo-node :outcome4 :outcome2)
      (send sm-sub :add-transition bar-node foo-node :outcome2))

    sm-top ))

(send (smach-nested) :execute nil)
```

### The Code Explained
```lisp
#!/usr/bin/env roseus
(load "package://roseus_smach/src/state-machine.l")
```
Same as in the previous example, this line imports state-machine class, state class, and transition class.

```lisp
  (let ((sm-top (instance state-machine :init))
        (sm-sub (instance state-machine :init)))
```
This line creates two of state-machine instance. The `sm-sub` is a state machine but also **acting like a state (or a node)** in `sm-top`. **This is easy to understand if you think that a state machine as a function**: when the state machine is called, or executed, state machine does some processing and eventually returns the goal state as its return value just like a function.

```lisp
    (send sm-top :add-node (instance state :init "SUB" sm-sub))
```
The `sm-sub` instance is hooked as a node in `sm-top` with name of `"SUB"` in this line. This line also indicates that you can add a state machine as a state just like a function.

```lisp
    (send sm-top :add-node (instance state :init "BAS" 'func-bas))
    (send sm-top :goal-state :outcome5)
    (send sm-top :start-state "BAS")
    (send sm-top :add-transition "BAS" "SUB" :outcome3)
```
These lines define another node, goal-state, start-state, and transition in the `sm-top`.
```lisp
    (send sm-top :add-transition "SUB" :outcome5 :outcome4)
```
Remember that **the goal state(s) of state-machine does nothing and returns its name**. So the outcome (or the return value) of the `sm-sub` is its name of goal state(s).
If you look further, the goal state of `sm-sub` is `:outcome4`, so the return value of `sm-sub` is `:outcome4`. Therefore this line adds transition of `from "SUB" to :outcome5 when "SUB" node returns :outcome4`.

```lisp
  (let ((foo-node (instance state :init "FOO" 'func-foo))
        (bar-node (instance state :init "BAR" 'func-bar)))
      (send sm-sub :add-node foo-node)
      (send sm-sub :add-node bar-node)
      (send sm-sub :goal-state :outcome4)
      (send sm-sub :start-state foo-node)
      (send sm-sub :add-transition foo-node bar-node :outcome1)
      (send sm-sub :add-transition foo-node :outcome4 :outcome2)
      (send sm-sub :add-transition bar-node foo-node :outcome2))
```
These lines define the behavior of `sm-sub` in detail just like the previous simple state machine example. Note that `(send sm-sub :goal-state :outcome4)` not only defines the goal state, but also defines the return value of its state machine.

```lisp
(send (smach-nested) :execute nil)
```
Finally, the `sm-top` is executed here.
