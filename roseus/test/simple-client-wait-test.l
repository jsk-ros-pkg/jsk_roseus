#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/cturtle/stacks/common/actionlib/test/test_client_wait_test.cpp
;;;

(load "unittest.l")
(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "actionlib")
(ros::roseus-add-srvs "roscpp")

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
    (unless (send c :wait-for-result)
      (warning-message "Goal didn't finish"))

    (assert (equal (send c :get-state) actionlib_msgs::succeeded))
    (assert (string= (send c :get-goal-status-text)
                          "The ref server has succeeded"))

    (send goal :goal :goal 4)
    (print (send c :send-goal-and-wait goal :timeout 2))
    (assert (equal (send c :get-state) actionlib_msgs::preempted))
    ))

(ros::roseus "simple_action_client")

(ros::service-call "~set_logger_level"
                   (instance roscpp::SetLoggerLevelRequest
                             :init :logger "ros" :level "DEBUG"))

(run-all-tests)
(exit)



