#!/usr/bin/env roseus
;;;
;;; ROS 2 version of add-two-ints-client.l
;;; Reduced iterations and shorter sleep for faster testing
;;; NIL-safe: handles service-call returning NIL on failure
;;;
(ros::load-ros-manifest "roseus")

(ros::roseus "add_two_ints_client")

(ros::wait-for-service "add_two_ints")
(dotimes (i 20)
  (setq req (instance roseus::AddTwoIntsRequest :init))
  (send req :a (random 10))
  (send req :b (random 20))
  (setq before (ros::time-now))
  (case (mod i 3)
    (0 (setq res (ros::service-call "add_two_ints" req t)))
    (1 (setq res (ros::service-call "add_two_ints" req nil)))
    (2 (setq res (ros::service-call "add_two_ints" req))))
  (setq after (ros::time-now))
  (if res
      (ros::ros-info "~d + ~d = ~d (~A sec)" (send req :a) (send req :b) (send res :sum) (send (ros::time- after before) :to-sec))
    (ros::ros-warn "service call failed for ~d + ~d" (send req :a) (send req :b)))
  (unix::usleep (* 200 1000)))  ;; 200ms instead of 1000ms

(exit)
