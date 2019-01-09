#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;
(ros::load-ros-manifest "roseus")

;;;
;;;
(ros::roseus "add_two_ints_client")

(when (setq *arguments* (remove-if #'(lambda (x) (and (substringp " __" x) (substringp ":=" x))) (member "add-two-ints-client.l" lisp::*eustop-argument* :test #'substringp)))
  (cond ((= (length *arguments*) 1)
	 (setq x '(random 10)
	       y '(random 20)))
	((= (length *arguments*) 3)
	 (setq x '(read-from-string (elt *arguments* 1))
	       y '(read-from-string (elt *arguments* 2))))
	(t
	 (ros::ros-error "Usage: ~A [x y]~%" (elt *arguments* 0))
	 (exit 1))))

(ros::wait-for-service "add_two_ints")
(dotimes (i 100)
  (setq req (instance roseus::AddTwoIntsRequest :init))
  (send req :a (eval x))
  (send req :b (eval y))
  (setq before (ros::time-now))
  (case (mod i 3)
    (0 (setq res (ros::service-call "add_two_ints" req t)))
    (1 (setq res (ros::service-call "add_two_ints" req nil)))
    (2 (setq res (ros::service-call "add_two_ints" req))))
  (setq after (ros::time-now))
  (ros::ros-info "~d + ~d = ~d~ (~A sec)" (send req :a) (send req :b) (send res :sum) (send (ros::time- after before) :to-sec))
  (unix:sleep 1))




