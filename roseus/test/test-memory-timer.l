#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::roseus "test_memory_timer")
(ros::rate 200)

(defclass my-cb :slots (done))
(defmethod my-cb
  (:update (event) (setq done t))
  (:get () done))

(defun test-once ()
  (let ((tm (instance my-cb)))
    (ros::create-timer 0.001 #'send tm :update :oneshot t)
    (while (not (send tm :get))
      (ros::spin-once)
      (ros::sleep))))

(deftest test-timer ()
  (dotimes (i 3000)
    (if (zerop (mod i 100)) (ros::ros-warn "timer: ~A~%" i))
    (test-once)))

(run-all-tests)
(exit)
