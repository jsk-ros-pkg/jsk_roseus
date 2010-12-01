#!/usr/bin/env roseus

(ros::roseus "test_publish_marker")

(defvar *pub-cylinder-topic* "/cylinder_marker")
(defvar *pub-cube-topic* "/cube_marker")
(defvar *pub-sphere-topic* "/sphere_marker")

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun test-pub ()
  (make-irtviewer)
  (ros::rate 10)
  (let ((cyl (make-cylinder 100 200))
	(cb (make-cube 100 100 200 :pos (float-vector 200 0 0)))
    	(sp (make-sphere 100 :pos (float-vector -200 0 0))))
    (send cyl :translate (float-vector 0 0 -100))
    (objects (list cyl cb sp))
    (do-until-key
     (let* ((header (instance roslib::header :init
                              :stamp (ros::time-now)
                              :frame_id "/test_id"))
            (cyl-msg (cylinder->marker-msg cyl header))
	    (cb-msg (cube->marker-msg cb header
				      :color (float-vector 0 1 0) :alpha 0.5))
	    (sp-msg (sphere->marker-msg sp header
					:color (float-vector 0 0 1) :alpha 0.5))
	    )
       (send *tfb* :send-transform (make-coords) "/tmp" "/test_id")
       (ros::publish *pub-cylinder-topic* cyl-msg)
       (ros::publish *pub-cube-topic* cb-msg)
       (ros::publish *pub-sphere-topic* sp-msg)
       (send cyl :rotate (deg2rad 1) :x)
       (send cb :rotate (deg2rad 1) :x)
       (send *irtviewer* :draw-objects)
       (x::window-main-one)
       (ros::spin-once)
       (ros::sleep)
       ))))

(ros::advertise *pub-cylinder-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-cube-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-sphere-topic* visualization_msgs::Marker 5)
(test-pub)
