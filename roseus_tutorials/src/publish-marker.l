#!/usr/bin/env roseus

(ros::roseus "test_publish_marker")

(defvar *pub-cylinder-topic* "/cylinder_marker")
(defvar *pub-cube-topic* "/cube_marker")
(defvar *pub-sphere-topic* "/sphere_marker")
(defvar *pub-line-topic* "/line_marker")
(defvar *pub-face-topic* "/face_marker")
(defvar *pub-all-topic* "/all_marker_array")

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun test-pub ()
  (make-irtviewer)
  (ros::rate 10)
  (let ((cyl (make-cylinder 100 200))
	(cb (make-cube 100 100 200 :pos (float-vector 200 0 0)))
    	(sp (make-sphere 100 :pos (float-vector -200 0 0)))
        (l (list #f(100 0 200) #f(-81 59 200) #f(31 -95 200) #f(31 95 200) #f(-81 -59 200) #f(100 0 200)))
        (f (make-face-from-vertices (list (list #f(100 0 -200)) (list #f(31 95 -200)) (list #f(-81 59 -200)) (list #f(-81 -59 -200)) (list #f(31 -95 -200)) (list #f(100 0 -200))))))
    (send cyl :translate (float-vector 0 0 -100))
    (objects (list cyl cb sp))
    (do-until-key
     (let* ((header (instance std_msgs::header :init
                              :stamp (ros::time-now)
                              :frame_id "/test_id"))
            (cyl-msg (cylinder->marker-msg cyl header))
	    (cb-msg (cube->marker-msg cb header
				      :color (float-vector 0 1 0) :alpha 0.5))
	    (sp-msg (sphere->marker-msg sp header
					:color (float-vector 0 0 1) :alpha 0.5))
            (l-msg (line->marker-msg l header
                                     :color (float-vector 1 1 0) :alpha 0.5))
            (f-msg (faces->marker-msg (list f) header
                                      :color (float-vector 0 1 1)))
	    (all-msg (instance visualization_msgs::MarkerArray :init))
	    )
       (send *tfb* :send-transform (make-coords) "/tmp" "/test_id")

       ;; for single msg
       (ros::publish *pub-cylinder-topic* cyl-msg)
       (ros::publish *pub-cube-topic* cb-msg)
       (ros::publish *pub-sphere-topic* sp-msg)
       (ros::publish *pub-line-topic* l-msg)
       (ros::publish *pub-face-topic* f-msg)

       ;; for array msg
       ;; you shold set up namespace (:ns ) or id (:id) for marker array
       (send cyl-msg :ns "test_cyl")
       (send cb-msg :ns "test_cb")
       (send sp-msg :ns "test_sp")
       (send l-msg :ns "test_l")
       (send f-msg :ns "test_f")
       (send all-msg :markers (list cyl-msg cb-msg sp-msg l-msg f-msg))
       (ros::publish *pub-all-topic* all-msg)

       (send cyl :rotate (deg2rad 1) :x)
       (send cb :rotate (deg2rad 1) :x)

       (send *irtviewer* :draw-objects)
       (x::window-main-one)
       (ros::spin-once)
       (ros::sleep)
       (print header)
       ))))

(ros::advertise *pub-cylinder-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-cube-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-sphere-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-line-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-face-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-all-topic* visualization_msgs::MarkerArray 5)
(test-pub)
