#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_client.py
;;;
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "actionlib_tutorials")
;;;
(when (member "--gtest_output=xml:" lisp::*eustop-argument* :test #'substringp)  ;; for test-fibonacci.launch
  (load "unittest.l")
  (init-unit-test))

;;;
;;;
(ros::roseus "fibonacci_client")
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

(setq count 0) ;; for test-fibonaci.launch
(do-until-key
 (when (boundp '*unit-test*)
   (warning-message 2 "test-fibonacci ~A/~A~%" count 1000)
   (incf count)
   (when (> count 1)
     (send *unit-test* :init-result "fibonacci-client")
     (send *unit-test* :print-result)
     (exit 0))
 (fibonacci-client))


