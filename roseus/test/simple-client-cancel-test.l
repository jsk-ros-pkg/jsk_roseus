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

(deftest test-client-cancel-by-myself ()
  (let (c1 g1)
    ;;
    ;; send goal and cancel by myself
    ;;
    (setq c1 (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server~%")
    (send c1 :wait-for-server)

    (warning-message 1 "send-goal 1~%")
    (setq g1 (instance actionlib::TestActionGoal :init))
    (send g1 :goal :goal 4) ;; 100 sec goal
    (send c1 :send-goal g1)
    
    (unix:sleep 1)
    (warning-message 1 "cancel-goal 1~%")
    (send c1 :cancel-goal)

    ;; as of 1.6.1, wait-for-result returns fail if a goal is canceled
    (warning-message 1 "wait-for-result 1~%")
    (unless (send c1 :wait-for-result)
      (warning-message 1 "wail-for-result 1 failed...~%")
      ;; if preempted that's what we expected
      (cond ((equal actionlib_msgs::GoalStatus::*PREEMPTED*
                    (send c1 :get-state))
             (warning-message 1 "goal is preempted, as expected...~%")
             )
            (t
             (assert nil "Goal 1 didn't finish~%"))))
    ))

(deftest test-client-cancel-by-others ()
  (let (c1 g1)
    ;;
    ;; send goal and cancel by others
    ;;
    (setq c1 (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server~%")
    (send c1 :wait-for-server)

    (warning-message 1 "send-goal 1~%")
    (setq g1 (instance actionlib::TestActionGoal :init))
    (send g1 :goal :goal 4) ;; 100 sec goal
    (send c1 :send-goal g1)
    
    (unix:sleep 1)
    ;; emuree someone send cancel goal
    (warning-message 1 "cancel-goal 1~%")
    (ros::publish (format nil "~A/cancel" (send c1 :name-space))
                          (instance actionlib_msgs::GoalId :init
                                    :id (send g1 :goal_id :id)))

    (warning-message 1 "wait-for-result 1~%")
    (unless (send c1 :wait-for-result)
      (warning-message 1 "wail-for-result 1 failed...~%")
      ;; if preempted that's what we expected
      (cond ((equal actionlib_msgs::GoalStatus::*PREEMPTED*
                    (send c1 :get-state))
             (warning-message 1 "goal is preempted, as expected...~%")
             )
            (t
             (assert nil "Goal 1 didn't finish~%"))))

    ))

(deftest test-client-send-goal-twice-with-one-instance ()
  (let (c1 g1)
    ;;
    ;; send goal twice from one instance
    ;;
    (setq c1 (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server~%")
    (send c1 :wait-for-server)

    (warning-message 1 "send-goal 1~%")
    (setq g1 (instance actionlib::TestActionGoal :init))
    (send g1 :goal :goal 4) ;; 100 sec goal
    (send c1 :send-goal g1)

    ;; 
    (unix:sleep 1)
    (warning-message 1 "send-goal 1 again~%")
    (setq g1 (instance actionlib::TestActionGoal :init))
    (send g1 :goal :goal 1) ;; 1 sec goal
    (send c1 :send-goal g1)

    (warning-message 1 "wait-for-result 1~%")
    (unless (send c1 :wait-for-result)
      (warning-message 1 "wail-for-result 1 failed...~%")
      ;; if preempted that's what we expected
      (assert nil "Goal 1 didn't finish~%"))
    ))

(deftest test-client-send-goal-twice-with-two-instance ()
  (let (c1 c2 g1 g2)
    ;;
    ;; send goal twice from two instance
    ;;
    (setq c1 (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server 1~%")
    (send c1 :wait-for-server)

    (setq g1 (instance actionlib::TestActionGoal :init))
    (warning-message 1 "send-goal 1~%")
    (send g1 :goal :goal 4) ;; 100 sec goal
    (send c1 :send-goal g1)
    (warning-message 1 "sent goal 1 (~A)~%" (send g1 :goal_id :id))
    (unix:sleep 1)
    ;;
    (setq c2 (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server 2~%")
    (send c2 :wait-for-server)

    (setq g2 (instance actionlib::TestActionGoal :init))
    (warning-message 1 "send-goal 2~%")
    (send g2 :goal :goal 1) ;; 1 sec goal
    (send c2 :send-goal g2)
    (warning-message 1 "sent goal 2 (~A)~%" (send g2 :goal_id :id))

    (warning-message 1 "wait-for-result 1 (~A)~%" (send g1 :goal_id :id))
    (unless (send c1 :wait-for-result :timeout 10)
      (warning-message 1 "wail-for-result 1 failed...~%")
      ;; if preempted that's what we expected
      (cond ((equal actionlib_msgs::GoalStatus::*PREEMPTED*
                    (send c1 :get-state))
             (warning-message 1 "goal is preempted, as expected...~%")
             )
            (t
             (assert nil "Goal 1 didn't finish~%"))))

    (warning-message 1 "wait-for-result 2 (~A)~%" (send g2 :goal_id :id))
    (unless (send c2 :wait-for-result :timeout 10)
      (warning-message 1 "wail-for-result 2 failed...~%")
      (assert nil "Goal 2 didn't finish~%"))
    ))

(deftest test-client-send-one-goal-with-two-instance ()
  (let (c1 c2 g1 g2)
    ;;
    ;; send goal twice from two instance
    ;;
    (setq c1 (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server 1~%")
    (send c1 :wait-for-server)

    (setq g1 (instance actionlib::TestActionGoal :init))
    (warning-message 1 "send-goal 1~%")
    (send g1 :goal :goal 4) ;; 10 sec to goal return set aborted
    (send c1 :send-goal g1)
    (warning-message 1 "sent goal 1 (~A)~%" (send g1 :goal_id :id))
    (unix:sleep 1)
    ;;
    (setq c2 (instance ros::simple-action-client :init
                      "reference_action" actionlib::TestAction))
    (warning-message 1 "wait-for-server 2~%")
    (send c2 :wait-for-server)
    ;; goal send by c1 is taken by c2, thus call (send c2 :wait-for-result) never retuns, so we need actinlib do not return from :wait-for-result, if that goal is overridden from different instance in same roseus process  hack (https://github.com/jsk-ros-pkg/jsk_roseus/pull/519#issuecomment-303000720)

    ;;
    (warning-message 1 "wait-for-result 1 (~A)~%" (send g1 :goal_id :id))
    (unless (send c1 :wait-for-result :timeout 20) ;; c1 is retuen after 10 sec
      (warning-message 1 "wail-for-result 1 failed...~%")
      ;; if the goal is aborted that's what we expected
      (cond ((equal actionlib_msgs::GoalStatus::*ABORTED*
                    (send c1 :get-state))
             (warning-message 1 "goal is aborted, as expected...~%")
             )
            (t
             (assert nil "Goal 1 didn't finish~%"))))
    ))

(setq *dispose* nil)
(defclass ros::simple-action-client-dispose
  :super ros::simple-action-client)
(defmethod ros::simple-action-client-dispose
  (:dispose
   ()
   (setq *dispose* t)
   (format *error-output* "~C[3~Cm[ WARN] dispose~%~C[0m" #x1b 51 #x1b)
   ))

(deftest test-client-dispose ()
  (let (c1 g1)
    (dotimes (i 4)
      (setq c1 (instance ros::simple-action-client-dispose :init
                         "reference_action" actionlib::TestAction))

      (warning-message 1 "wait-for-server ~A ~A~%" i c1)
      (send c1 :wait-for-server)

      (setq g1 (instance actionlib::TestActionGoal :init))
      (warning-message 1 "send-goal ~A~%" i)
      (send g1 :goal :goal 1) ;; 1 sec goal
      (send c1 :send-goal g1)
      (warning-message 1 "sent goal ~A (~A)~%" i (send g1 :goal_id :id))
      )
    (warning-message 1 "wait-for-result~%")

    (warning-message 1 "wait-for-result (~A)~%" (send g1 :goal_id :id))
    (unless (send c1 :wait-for-result :timeout 10)
      (warning-message 1 "wail-for-result failed...~%")
      (assert nil "Goal 2 didn't finish~%"))

    (dotimes (i 10)
      (warning-message 1 "wait-for-result ~A~%" i)
      (sys::gc)
      (unix:usleep (* 500 1000)))
    (warning-message 1 ";;; :dispose is not called ~A~%" *dispose*)
    (warning-message 1 ";;; because of (set (gentemp \"ACTIONLIBCLIENT\" (find-package \"ROSACTIONLIB\")) self) and https://github.com/jsk-ros-pkg/jsk_roseus/pull/525~%")
    ;; (assert *dispose*) ;; this failed because of (set (gentemp "ACTIONLIBCLIENT" (find-package "ROSACTIONLIB")) self) and #525
    ))

(ros::roseus "simple_action_client_cancel_test" :anonymous nil)

;(ros::service-call "~set_logger_level"
;                   (instance roscpp::SetLoggerLevelRequest
;                             :init :logger "ros.roseus" :level "DEBUG"))

(run-all-tests)
(exit)



