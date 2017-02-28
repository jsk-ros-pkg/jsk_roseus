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
