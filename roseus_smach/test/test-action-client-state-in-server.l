#!/usr/bin/env roseus
;; test-smach-action-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus-add-msgs "actionlib_tutorials")

(require :unittest "lib/llib/unittest.l")

(ros::roseus "test_smach_action_server")

(init-unit-test)

(deftest test-action-client-state-in-server ()
  (setq *ac* (instance ros::simple-action-client :init
                       "/fibonacci_test"
                       actionlib_tutorials::fibonacciaction))
  (assert (send *ac* :wait-for-server 10)
          "/fibonnaci_test action is initialized")

  (let ((goal (instance actionlib_tutorials::fibonacciactiongoal :init)))
    (send goal :goal :order 5)
    (send *ac* :send-goal goal))
  (unix:sleep 1)
  (send *ac* :spin-once)
  (assert (eq (send *ac* :get-state)
              actionlib_msgs::GoalStatus::*active*)
          "/fibonacci_test action is active")
  (assert (send *ac* :wait-for-result :timeout 30)
          "/fibonacci_test action wait-for-result returns t")
  (warning-message 2 "get-state: ~A~%" (send *ac* :get-state))
  (assert (eq (send *ac* :get-state)
              actionlib_msgs::GoalStatus::*succeeded*)
          "/fibonacci_test action state is *succeeded*")
  (print-ros-msg (send *ac* :get-result))
  (assert (and (send (send *ac* :get-result) :sequence)
          "/fibonacci_test action returns result")))

(deftest test-smach-action-server-cancel ()
  (setq *ac* (instance ros::simple-action-client :init
                       "/fibonacci_test"
                       actionlib_tutorials::fibonacciaction))
  (assert (send *ac* :wait-for-server 10)
          "/fibonnaci_test action is initialized")

  (let ((goal (instance actionlib_tutorials::fibonacciactiongoal :init)))
    (send goal :goal :order 30)
    (send *ac* :send-goal goal))
  (unix:sleep 1)
  (send *ac* :spin-once)
  (assert (eq (send *ac* :get-state)
              actionlib_msgs::GoalStatus::*active*)
          "/fibonacci_test action is active")

  (unix:sleep 3)
  (send *ac* :cancel-goal)
  (warning-message 2 ":cancel-goal is sent~%")
  (send *ac* :wait-for-result :timeout 30)
  (send *ac* :spin-once)
  (warning-message 2 "get-state: ~A~%" (send *ac* :get-state))
  ;; see http://wiki.ros.org/actionlib/DetailedDescription
  (assert (memq (send *ac* :get-state)
                (list
                 actionlib_msgs::GoalStatus::*aborted*
                 actionlib_msgs::GoalStatus::*preempted*
                 actionlib_msgs::GoalStatus::*preempting*))
          (format nil "/fibonacci_test action state ~A is *preempted* or *preempting*"
           (send *ac* :get-state)))
  (print-ros-msg (send *ac* :get-result))
  (assert (integer-vector-p (send (send *ac* :get-result) :sequence))
          "/fibonnaci_test action returns result")
  )


(run-all-tests)
(exit)
