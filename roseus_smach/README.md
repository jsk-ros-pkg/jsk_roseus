roseus_smach
============

This package includes euslisp imprementation of state machine and [smach](http://wiki.ros.org/smach).

## requirements

- [roseus](http://wiki.ros.org/roseus)
- [smach](http://wiki.ros.org/smach)
- [smach_viewer](http://wiki.ros.org/smach_viewer) (optional for visualization)

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
;;
;; 3 type samples of State machine from SMACH tutorials
;;
(load "package://roseus_smach/src/state-machine.l")

;;
;; sample 1: simple state machine
;;
(setq count 0)
(defun func-foo (&rest args)
  (format t "Execute state FOO~%")
  (cond ((< count 3) (incf count) :outcome1)
	(t :outcome2)))
(defun func-bar (&rest args)
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
```
### The Code Explained
```lisp
(load "package://roseus_smach/src/state-machine.l")
```
This line imports state-machine class, state class, and transition class.



```lisp
(defun func-foo (&rest args)
  (format t "Execute state FOO~%")
  (cond ((< count 3) (incf count) :outcome1)
	(t :outcome2)))
(defun func-bar (&rest args)
  (format t "Execute state BAR~%")
  :outcome2)
```
These lines define some functions that will be hooked to state. Note that the return value of the function is equal to the transition (or the action) of the state.


```lisp
(sm (instance state-machine :init))
```
This line creates state-machine instance. 
Noe that you can execute the state machine with `(exec-state-machine sm)` then you can view and check it with [smach_viewer](http://wiki.ros.org/smach_viewer). The function `exec-state-machine ` is defined in [here](https://github.com/jsk-ros-pkg/jsk_roseus/blob/master/roseus_smach/src/state-machine-utils.l#L3). 


```lisp
(send sm :add-node (instance state :init :FOO 'func-foo))
```
This line creates a new state and add the node to the state machine. When you create new state instance, you can name the state and hook the state to a function. In this case, a new state has name `:FOO` and hooked with `func-foo`.  

```lisp
(send sm :goal-state (list :outcome4 :outcome5))
(send sm :start-state :FOO)
```
These lines define goal state(s) and start state(s) to the state machine. You can create multiple start staets as a list of state, also you can define multiple goal state. Goal state(s) do nothing and retuns its name.   Note that **a state machine is also a state**. So the goal state can be considered as the outcome of the state machine. 


```lisp
(send sm :add-transition :FOO :BAR :outcome1)
 ```
 This line defines transtion. The argments of `:add-transition method` is `TO, FROM, OUTCOME`. So this line means that "adding transition from FOO to BAR, when FOO node returns outcome1". 
 
