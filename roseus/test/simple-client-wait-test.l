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

;;
;; check roseus actionlib status callback, if staus queue is long, :spin-once takes long time
;; https://github.com/jsk-ros-pkg/jsk_roseus/issues/396
;;
(defclass ros::simple-action-client-test
  :super  ros::simple-action-client
  :slots (status-cb-num))
(defmethod ros::simple-action-client-test
  (:init (&rest args) (setq status-cb-num 0) (send-super* :init args))
  (:status-cb-num () status-cb-num)
  (:goal-status-cb
   (msg)
   (incf status-cb-num)
   (send-super :goal-status-cb msg)))
(deftest test-client-wait ()
  (let (c goal tm0 tm1)
    (setq c (instance ros::simple-action-client-test :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server~%")
    (send c :wait-for-server)

    (setq goal (instance actionlib::TestActionGoal :init))
    (send goal :goal :goal 1)
    (send c :send-goal goal)

    ;; wait for 10 sec
    (warning-message 1 "sleep 10~%")
    (unix:sleep 10)
    (warning-message 1 "wait-for-result~%")
    (setq tm0 (ros::time-now))
    (unless (send c :wait-for-result :timeout 10)
      (warning-message 1 "Goal didn't finish"))
    (setq tm1 (ros::time-now))
    (warning-message 1 "status-cb-num ~A~%" (send c :status-cb-num))
    (warning-message 1 "it takes ~A[sec] to wait for result~%" (send (ros::time- tm1 tm0) :to-sec))
    (assert (< (send (ros::time- tm1 tm0) :to-sec) 0.2) "it should be quick enough")

    (assert (equal (send c :get-state) actionlib_msgs::GoalStatus::*succeeded*))
    (assert (string= (send c :get-goal-status-text)
                          "The ref server has succeeded"))
    ))

(ros::roseus "simple_action_client")

(ros::service-call "~set_logger_level"
                   (instance roscpp::SetLoggerLevelRequest
                             :init :logger "ros" :level "DEBUG"))

(run-all-tests)
(exit)



