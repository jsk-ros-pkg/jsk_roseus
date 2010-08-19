#!/usr/bin/env runeus
;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_client.py
;;;

(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "actionlib_tutorials")
;;;

;;;
;;;
(roseus "fibonacci_client")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(defun fibonacci-client ()
  (let (c goal)
    (setq c (instance ros::simple-action-client :init
                      "fibonacci" actionlib_tutorials::FibonacciAction))
    (warning-message 1 "wait-for-server~%")
    (send c :wait-for-server)
    (setq goal (instance actionlib_tutorials::FibonacciActionGoal :init))
    (send goal :goal :order 10)
    (send c :send-goal goal)

    (warning-message 1 "wait-for-result~%")
    (send c :wait-for-result)
    (warning-message 1 (format nil ";; result -> ~A~%"  (send (send c :get-result) :sequence)))
    ))

(do-until-key
 (fibonacci-client))


