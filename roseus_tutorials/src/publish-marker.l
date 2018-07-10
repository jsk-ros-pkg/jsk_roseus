#!/usr/bin/env roseus

(ros::roseus "test_publish_marker")

(defvar *pub-cylinder-topic* "/cylinder_marker")
(defvar *pub-cube-topic* "/cube_marker")
(defvar *pub-sphere-topic* "/sphere_marker")
(defvar *pub-line-topic* "/line_marker")
(defvar *pub-face-topic* "/face_marker")

(defvar *pub-cylinder-topic-2* "/cylinder_marker_2")
(defvar *pub-cube-topic-2* "/cube_marker_2")
(defvar *pub-sphere-topic-2* "/sphere_marker_2")
(defvar *pub-line-topic-2* "/line_marker_2")
(defvar *pub-face-topic-2* "/face_marker_2")

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
            (header-2 (instance std_msgs::header :init
                              :stamp (ros::time-now)
                              :frame_id "/test_id_2"))
            (cyl-msg (cylinder->marker-msg cyl header
				      :color (float-vector 1 0 0) :alpha 0.5))
	    (cb-msg (cube->marker-msg cb header
				      :color (float-vector 0 1 0) :alpha 0.5))
	    (sp-msg (sphere->marker-msg sp header
					:color (float-vector 0 0 1) :alpha 0.5))
            (l-msg (line->marker-msg l header
                                     :color (float-vector 1 1 0) :alpha 0.5))
            (f-msg (faces->marker-msg (list f) header
                                      :color (float-vector 0 1 1)))
            (cyl-msg-2 (eusobj->marker-msg cyl header-2
				      :color (float-vector 1 0 0) :alpha 0.5))
	    (cb-msg-2 (eusobj->marker-msg cb header-2
				      :color (float-vector 0 1 0) :alpha 0.5))
	    (sp-msg-2 (eusobj->marker-msg sp header-2
					:color (float-vector 0 0 1) :alpha 0.5))
            (l-msg-2 (eusobj->marker-msg l header-2
                                     :color (float-vector 1 1 0) :alpha 0.5))
            (f-msg-2 (eusobj->marker-msg (list f) header-2
                                      :color (float-vector 0 1 1)))
	    (all-msg (instance visualization_msgs::MarkerArray :init))
	    )
       (send *tfb* :send-transform (make-coords) "/tmp" "/test_id")
       (send *tfb* :send-transform (make-coords :pos #f(500 0 0)) "/tmp" "/test_id_2")

       ;; for single msg
       (ros::publish *pub-cylinder-topic* cyl-msg)
       (ros::publish *pub-cube-topic* cb-msg)
       (ros::publish *pub-sphere-topic* sp-msg)
       (ros::publish *pub-line-topic* l-msg)
       (ros::publish *pub-face-topic* f-msg)

       (ros::publish *pub-cylinder-topic-2* cyl-msg-2)
       (ros::publish *pub-cube-topic-2* cb-msg-2)
       (ros::publish *pub-sphere-topic-2* sp-msg-2)
       (ros::publish *pub-line-topic-2* l-msg-2)
       (ros::publish *pub-face-topic-2* f-msg-2)

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
(ros::advertise *pub-cylinder-topic-2* visualization_msgs::Marker 5)
(ros::advertise *pub-cube-topic-2* visualization_msgs::Marker 5)
(ros::advertise *pub-sphere-topic-2* visualization_msgs::Marker 5)
(ros::advertise *pub-line-topic-2* visualization_msgs::Marker 5)
(ros::advertise *pub-face-topic-2* visualization_msgs::Marker 5)
(ros::advertise *pub-all-topic* visualization_msgs::MarkerArray 5)
(test-pub)
