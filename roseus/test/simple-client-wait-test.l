#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/cturtle/stacks/common/actionlib/test/test_client_wait_test.cpp
;;;

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "actionlib")
(ros::load-ros-manifest "roscpp")

(init-unit-test)
;;;
;;;

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(deftest test-client ()
  (let (c goal)
    (setq c (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server~%")
    (send c :wait-for-server)

    (setq goal (instance actionlib::TestActionGoal :init))
    (send goal :goal :goal 1)
    (send c :send-goal goal)
    (warning-message 1 "wait-for-result~%")
    (unless (send c :wait-for-result :timeout 10)
      (warning-message 1 "Goal didn't finish"))

    (assert (equal (send c :get-state) actionlib_msgs::GoalStatus::*succeeded*))
    (assert (string= (send c :get-goal-status-text)
                          "The ref server has succeeded"))

    (send goal :goal :goal 4)
    (warning-message 1 "send-goal-and-wait~%")
    (print (send c :send-goal-and-wait goal :timeout 2) *error-output*)
    (assert (equal (send c :get-state) actionlib_msgs::GoalStatus::*preempted*))
    ))

(ros::roseus "simple_action_client")

(ros::service-call "~set_logger_level"
                   (instance roscpp::SetLoggerLevelRequest
                             :init :logger "ros" :level "DEBUG"))

(run-all-tests)
(exit)



