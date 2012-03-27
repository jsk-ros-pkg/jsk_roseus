#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/talker_listener_test.py
;;;

(require :unittest "lib/llib/unittest.l")

(ros::load-ros-manifest "roseus")

;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))
(init-unit-test)

;; method call
(defclass string-cb-class
  :super propertied-object
  :slots ())
(defmethod string-cb-class
  (:init () (ros::subscribe "chatter" std_msgs::string #'send self :string-cb))
  (:string-cb
   (msg)
   (print (list 'cb self (send msg :data)) *error-output*)
   (assert (substringp "hello world" (send msg :data))))
  )

(deftest test-talker-listener
  (ros::roseus "listener")
  (setq m (instance string-cb-class :init))

  (ros::rate 10)
  (dotimes (i 25)
    (ros::spin-once)
    (ros::sleep)
    (sys::gc)
    ))

(run-all-tests)
(exit)

