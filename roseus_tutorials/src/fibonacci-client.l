#!/usr/bin/env roseus
;;;
;;; euslisp version of https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient (ROS 1)
;;;
(ros::load-ros-manifest "example_interfaces")

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(ros::roseus "fibonacci_client")

(defun feedback-cb
    (msg)
  (format t "feedback sequence: ~a~%" (send msg :sequence)))

(setq *action-client* (instance ros::simple-action-client :init
                                "fibonacci" example_interfaces::Fibonacci))
(send *action-client* :wait-for-server)
(setq *goal* (instance example_interfaces::FibonacciGoal :init :order 10))
(send *action-client* :send-goal *goal* :feedback-cb #'feedback-cb)
(send *action-client* :wait-for-result)
(ros::ros-info "Result ~A" (send (send *action-client* :get-result) :sequence))
