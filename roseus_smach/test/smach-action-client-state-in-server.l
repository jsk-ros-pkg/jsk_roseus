#!/usr/bin/env roseus
;; smach-action-client-state-in-server.l
;; Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus-add-msgs "actionlib_tutorials")
(load "package://roseus_smach/sample/state-machine-ros-sample.l")

(ros::roseus "fibonacci_test")

(setq *sm* (smach-action-client-state))
(send *sm* :arg-keys :goal :cancel :result :feedback)
(setq *as* (instance state-machine-action-server :init
                     "/fibonacci_test" actionlib_tutorials::fibonacciaction
                     *sm*))
(send *as* :success-state '(:SUCCEED-STATE))

(while (ros::ok)
  (send *as* :worker)
  (ros::spin-once)
  (ros::sleep))
(exit)
