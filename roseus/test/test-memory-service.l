#!/usr/bin/env roseus
(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(ros::roseus "test_memory_subscriber")
(ros::rate 200)

(defclass my-srv :slots (req))
(defmethod my-srv
  (:update (_req)
   (setq req _req)
   (send _req :response))
  (:get () req))

(sys:make-thread 1)
(defun test-once (topic)
  (let ((srv (instance my-srv)))
    (ros::advertise-service topic std_srvs::Empty #'send srv :update)
    (ros::wait-for-service  topic)
    (sys:thread-no-wait #'ros::service-call topic (instance std_srvs::EmptyRequest :init))
    (while (not (send srv :get))
      (ros::spin-once))
    (ros::unadvertise-service topic)))

(deftest test-memory-service ()
  (dotimes (i 2000)
     (if (zerop (mod i 100)) (ros::ros-warn "service: ~A~%" i))
     (test-once "/roseus_test/srv")))

(run-all-tests)
(exit)
