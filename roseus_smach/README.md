#roseus_smach
- - -

This package includes euslisp imprementation of state machine and [smach](http://wiki.ros.org/smach).

## requirements

- [roseus](http://wiki.ros.org/roseus)
- [smach](http://wiki.ros.org/smach)
- [smach_viewer](http://wiki.ros.org/smach_viewer) (optional for visualization)

## sample

Sample codes are available on `sample` directory.

- `sample/state-machine-ros-sample.l`
  - simple state machine
  ![](https://gist.github.com/furushchev/9b1ed0aa57b47537cd2d/raw/3ad11bb4e5be0d50496c267b2fd38f18368948b0/smach-simple.gif)
  ```lisp
  roseus state-machine-ros-sample.l
  (smach-exec-simple)
  ```
  - nested state machine
  ![](https://gist.github.com/furushchev/9b1ed0aa57b47537cd2d/raw/3ad11bb4e5be0d50496c267b2fd38f18368948b0/smach-nested.gif)
  ```lisp
  roseus state-machine-ros-sample.l
  (smach-exec-nested)
  ```
  - state machine with userdata
  ![](https://gist.github.com/furushchev/9b1ed0aa57b47537cd2d/raw/3ad11bb4e5be0d50496c267b2fd38f18368948b0/smach-userdata.gif)
  ```lisp
  roseus state-machine-ros-sample.l
  (smach-exec-userdata)
  ```
  - state machine with parallel action execution
  ![](https://gist.github.com/furushchev/9b1ed0aa57b47537cd2d/raw/3ad11bb4e5be0d50496c267b2fd38f18368948b0/smach-parallel.gif)
  ```lisp
  roseus parallelstate-machine-sample.l
  (demo)
  ```
  
  
