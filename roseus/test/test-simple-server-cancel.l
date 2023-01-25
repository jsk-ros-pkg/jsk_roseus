#!/usr/bin/env roseus
;;;
;;;

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-package "actionlib")
;;;
;;;
(init-unit-test)

(defun cancel-cb (server goal)
  (send server :set-preempted))

(deftest test-server-cancel ()
  (let* ((server (instance ros::simple-action-server :init
                           "/cancel_test" actionlib::TestAction
                           :execute-cb #'cancel-cb))
         (client (instance ros::simple-action-client :init
                           "/cancel_test" actionlib::TestAction))
         (goal (instance actionlib::TestActionGoal :init)))

    (send client :wait-for-server)
    (send client :send-goal goal)
    ;; wait until the message is received
    (while (not (send server :is-active))
      (ros::sleep)
      (send server :spin-once))
    (send server :worker)
    (send client :spin-once)
    (ros::ros-info "action server returned with status: ~S" (send client :get-state))
    (assert (equal actionlib_msgs::GoalStatus::*PREEMPTED*
                   (send client :get-state)))))))

(ros::roseus "server_cancel")
(run-all-tests)

(exit)
