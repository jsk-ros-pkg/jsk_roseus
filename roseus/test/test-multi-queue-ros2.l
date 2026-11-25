#!/usr/bin/env roseus
;;
;; ROS 2 version of test-multi-queue.l
;; - Longer wait for DDS discovery (takes ~8s between processes)
;; - Tests that multiple subscribers on different topics receive messages
;;
;; $ ros2 topic pub -r 4 /a std_msgs/msg/String "{data: 'from a'}"
;; $ ros2 topic pub -r 4 /b std_msgs/msg/String "{data: 'from b'}"
;;
(require :unittest "lib/llib/unittest.l")

(ros::roseus-add-msgs "std_msgs")
(ros::roseus "test-multi-queue")

(setq *msg-a* nil)
(setq *msg-b* nil)

(defun callback-a (msg)
  (setq *msg-a* (send msg :data)))
(defun callback-b (msg)
  (setq *msg-b* (send msg :data)))

(ros::subscribe "/a" std_msgs::String #'callback-a)
(ros::subscribe "/b" std_msgs::String #'callback-b)

(init-unit-test)

(deftest test-multi-subscribe
  ;; Wait for DDS discovery (can take ~8 seconds between processes)
  (dotimes (i 40)
    (ros::spin-once)
    (when (and *msg-a* *msg-b*) (return))
    (unix::usleep (* 500 1000)))
  (assert *msg-a* (format nil "did not receive message on /a"))
  (assert *msg-b* (format nil "did not receive message on /b"))
  (when (and *msg-a* *msg-b*)
    (assert (string= *msg-a* "from a")
            (format nil "expected 'from a' but got '~A'" *msg-a*))
    (assert (string= *msg-b* "from b")
            (format nil "expected 'from b' but got '~A'" *msg-b*))))

(run-all-tests)

(exit)
