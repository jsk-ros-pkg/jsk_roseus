#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/cturtle/stacks/common/actionlib/test/simple_client.cpp
;;;

(load "unittest.l")
(ros::roseus-add-msgs "roslib")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "actionlib")
;;;
;;;
(init-unit-test)

;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(deftest test-simple-client ()
  (let (c goal)
    (setq c (instance ros::simple-action-client :init
                      "reference_simple_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server~%")
    (send c :wait-for-server)
    (setq goal (instance actionlib::TestActionGoal :init))
    (send goal :goal :goal 1)
    (send c :send-goal goal)
    (warning-message 1 "wait-for-result~%")
    (unless (send c :wait-for-result)
      (warning-message 1 "Goal didn't finish~%"))
    (assert-true (equal (send c :get-state) actionlib_msgs::succeeded))
    (assert-true (string= (send c :get-goal-status-text)
                          "The ref server has succeeded"))
    ;;
    (send goal :goal :goal 2)
    (send c :send-goal goal)
    (warning-message 1 "wait-for-result~%")
    (unless (send c :wait-for-result)
      (warning-message 1 "Goal didn't finish~%"))
    (assert-true (equal (send c :get-state) actionlib_msgs::aborted))
    (assert-true (string= (send c :get-goal-status-text)
                          "The ref server has aborted"))

    (send goal :goal :goal 3)
    (send c :send-goal goal)
    (warning-message 1 "wait-for-result~%")
    (unless (send c :wait-for-result)
      (warning-message 1 "Goal didn't finish~%"))
    ;; The simple server can't reject goals
    (assert-true (send c :get-state) actionlib_msgs::aborted)

    (setq saved-feedback nil)
    (send goal :goal :goal 9)
    (send c :send-goal goal :feedback-cb #'(lambda (x)
                                             (push x saved-feedback)
                                             (ros::ros-info "Got Feedback")))
    (unless (send c :wait-for-result)
      (warning-message 1 "Goal didn't finish~%"))
    (assert-true (equal (send c :get-state) actionlib_msgs::succeeded))
    (assert-true (equal (send (elt saved-feedback 0) :feedback :feedback) 9))

    (send c :cancel-all-goals)
    ))

(ros::roseus "simple_action_client")

(run-all-tests)
(exit)

