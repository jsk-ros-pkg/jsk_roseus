#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::roseus "test_memory_subscriber")
(ros::rate 200)

(defclass my-sub :slots (msg))
(defmethod my-sub
 (:update (_msg) (setq msg _msg))
 (:get () msg))

(defun test-once (topic msgtype)
 (let ((sub (instance my-sub)))
   (ros::subscribe topic msgtype #'send sub :update)
   (while (not (send sub :get))
     (ros::spin-once)
     (ros::sleep))
   (send sub :get)))

(deftest test-memory-subscriber ()
  (dotimes (i 2000)
     (if (zerop (mod i 100)) (ros::ros-warn "subscriber: ~A" i))
     (test-once "/roseus_test/msg" std_msgs::String)))

(run-all-tests)
(exit)
