#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/001_talker_listener
;;;

(ros::load-ros-manifest "roseus")
;;;

;;;
;;;
(ros::roseus "listener")
;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

;; callback function
;(defun string-cb (msg) (print (list 'cb (sys::thread-self) (send msg :data))))
;(ros::subscribe "chatter" std_msgs::string #'string-cb)

; lambda function
;(ros::subscribe "chatter" std_msgs::string
;                #'(lambda (msg) (ros::ros-info
;                                 (format nil "I heard ~A" (send msg :data)))))

;; method call
(defclass string-cb-class
  :super propertied-object
  :slots ())
(defmethod string-cb-class
  (:init () (ros::subscribe "chatter" std_msgs::string #'send self :string-cb))
  (:string-cb (msg) (print (list 'cb self (send msg :data)))))
(setq m (instance string-cb-class :init))

(do-until-key
 (ros::spin-once)
 ;;(sys::gc)
)
;(ros::spin)
