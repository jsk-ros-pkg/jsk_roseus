#!/usr/bin/env roseus

(setq sys::*gc-hook* #'(lambda (a b) (format *error-output* ";; gc ~A ~A~%" a b)))

;;
(ros::roseus "test_timer")
(ros::advertise "chatter1" std_msgs::string 1)
(ros::advertise "chatter2" std_msgs::string 1)
(ros::advertise "chatter3" std_msgs::string 1)
(ros::advertise "chatter4" std_msgs::string 1)
(ros::advertise "chatter5" std_msgs::string 1)

(defun cb1 (event)
  (let (msg)
    (ros::ros-info "~A" event)
    (setq msg (instance std_msgs::string :init))
    (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
    (ros::publish "chatter1" msg)
    ))
(defun cb2 (event)
  (let (msg)
    (setq msg (instance std_msgs::string :init))
    (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
    (ros::publish "chatter2" msg)
    ))
(ros::create-timer 0.1 #'cb1)
(ros::create-timer 0.1 #'cb2)

(ros::create-timer 0.1 #'(lambda (event) 
                          (let (msg)
                            (setq msg (instance std_msgs::string :init))
                            (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
                            (ros::publish "chatter3" msg)
                            )))

(defclass cb
  :super propertied-object
  :slots (topic))
(defmethod cb
  (:init (tpc) (setq topic tpc) self)
  (:call (event)
   (let (msg)
     (setq msg (instance std_msgs::string :init))
     (send msg :data (format nil "hello world ~a" (send (ros::time-now) :sec-nsec)))
     (ros::publish topic msg)
     )))
(setq ccb1 (instance cb :init "chatter4"))
(setq ccb2 (instance cb :init "chatter5"))
(ros::create-timer 0.1 #'send ccb1 :call)
(ros::create-timer 0.1 #'send ccb2 :call)
(ros::spin)




