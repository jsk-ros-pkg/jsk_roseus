#!/usr/bin/env roseus
;; test-mark-lock.l
;; Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :unittest "lib/llib/unittest.l")
(ros::roseus "test-mark-lock")

(init-unit-test)

(defun test-timer-cb (event)
  (ros::ros-info "timer callback")
  (setq *timer-callback-param* (ros::get-param "timer_param" t))
  (superequal t t) ;; function using mark_lock
  (setq *print-circle* t) (print 0) (setq *print-circle* nil) ;; function using mark_lock
  (ros::ros-info "timer callback end"))

(deftest test-mark-lock ()
  (ros::ros-info "Calling ros::get-param in timer-callback.")
  (ros::ros-warn "It may occurs dead lock if fail.")
  (setq *timer-callback-param* nil)
  (ros::create-timer 1.0 #'test-timer-cb)
  (setq start-time (ros::time-now))
  (while (ros::ok)
    (when (> (send (ros::time- (ros::time-now) start-time) :to-sec) 5.0)
      (return))
    (when *timer-callback-param* (return))
    (ros::sleep)
    (ros::spin-once))
  (assert *timer-callback-param* "test-mark-lock")
  t)

(run-all-tests)

(exit)

