#!/usr/bin/env roseus
;; test-action-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus-add-msgs "actionlib_tutorials")

(require :unittest "lib/llib/unittest.l")

(ros::roseus "test_action_server")

(init-unit-test)

(deftest test-smach-action-server ()
  (setq *ac* (instance ros::simple-action-client :init
                       "fibonacci"
                       actionlib_tutorials::fibonacciaction))
  (assert (send *ac* :wait-for-server 10)
          "/fibonnaci action is initialized")

  (let ((goal (instance actionlib_tutorials::fibonacciactiongoal :init)))
    (send goal :goal :order 5)
    (send *ac* :send-goal goal))
  (unix:sleep 1)
  (send *ac* :spin-once)
  (assert (eq (send *ac* :get-state)
              actionlib_msgs::GoalStatus::*active*)
          "/fibonacci action is active")
  (assert (send *ac* :wait-for-result :timeout 30)
          "/fibonacci action wait-for-result returns t")
  (warning-message 2 "get-state: ~A~%" (send *ac* :get-state))
  (assert (eq (send *ac* :get-state)
              actionlib_msgs::GoalStatus::*succeeded*)
          "/fibonacci action state is *succeeded*")
  (assert (and (send (send *ac* :get-result) :sequence)
          "/fibonnaci action returns result")))

(deftest test-smach-action-server-cancel ()
  (setq *ac* (instance ros::simple-action-client :init
                       "fibonacci"
                       actionlib_tutorials::fibonacciaction))
  (assert (send *ac* :wait-for-server 10)
          "/fibonnaci action is initialized")

  (let ((goal (instance actionlib_tutorials::fibonacciactiongoal :init)))
    (send goal :goal :order 30)
    (send *ac* :send-goal goal))
  (unix:sleep 1)
  (send *ac* :spin-once)
  (assert (eq (send *ac* :get-state)
              actionlib_msgs::GoalStatus::*active*)
          "/fibonacci action is active")

  (unix:sleep 3)
  (send *ac* :cancel-goal)
  (send *ac* :wait-for-result :timeout 30)
  (send *ac* :spin-once)
  (warning-message 2 "get-state: ~A~%" (send *ac* :get-state))
  (assert (memq (send *ac* :get-state)
                (list actionlib_msgs::GoalStatus::*preempted*
                      actionlib_msgs::GoalStatus::*preempting*))
          "/fibonacci action state is *preempting* or *preempted*")
  (print-ros-msg (send *ac* :get-result))
  (assert (> (length (send (send *ac* :get-result) :sequence)) 0)
          "/fibonnaci action returns result")
  )

(run-all-tests)
(exit)
