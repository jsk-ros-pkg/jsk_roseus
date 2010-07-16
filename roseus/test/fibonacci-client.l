#!/usr/bin/env runeus
;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_client.py
;;;
(unless (find "roseus/euslisp/" *load-path* :test #'substringp)
  (pushnew (format nil "~A/euslisp/"
                   (read-line (piped-fork "rospack find roseus")))
           *load-path* :test #'equal))
(load "roseus.l")
(load "actionlib.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-actionlibs "actionlib_tutorials")
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


