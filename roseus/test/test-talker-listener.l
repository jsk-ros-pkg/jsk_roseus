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
  :slots (msg-camep))
(defmethod string-cb-class
  (:init
    ()
    (setq msg-camep nil)
    (ros::subscribe "chatter" std_msgs::string #'send self :string-cb))
  (:string-cb
   (msg)
   (setq msg-camep t)
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
    )
  (assert (send m :get-val 'msg-camep))
  )

(deftest test-talker-listener-connection-header
  (ros::roseus "listener-conection-header")
  ;; callback function
  (defun string-cb (msg) (print (list 'cb (send msg :data) (send msg :connection-header)) *error-output*) (assert (send msg :connection-header)))
  (ros::subscribe "chatter" std_msgs::string #'string-cb)

  (ros::rate 10)
  (dotimes (i 25)
    (ros::spin-once)
    (ros::sleep))
  )

(run-all-tests)
(exit)

