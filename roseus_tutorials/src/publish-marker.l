#!/usr/bin/env roseus

(roseus "test_publish_marker")

(load-ros-manifest "test_publish_marker")

(defvar *pub-cylinder-topic* "/cylinder_marker")
(defvar *pub-cube-topic* "/cube_marker")
(defvar *pub-sphere-topic* "/sphere_marker")

(defun x-of-cube (cub)
  (let ((csg (send cub :csg)))
    (car (cdr (assoc :cube csg)))))

(defun y-of-cube (cub)
  (let ((csg (send cub :csg)))
    (cadr (cdr (assoc :cube csg)))))

(defun z-of-cube (cub)
  (let ((csg (send cub :csg)))
    (caddr (cdr (assoc :cube csg)))))

(defun height-of-cylinder (cyl)
  (let ((csg (send cyl :csg)))
    (caddr (assoc :cylinder csg))))

(defun radius-of-cylinder (cyl)
  (let ((csg (send cyl :csg)))
    (cadr (assoc :cylinder csg))))

(defun make-sphere (r &rest args)
  (let ((obj (make-gdome (make-icosahedron r))))
    (send obj :newcoords (apply #'make-coords args))
    (send obj :worldpos)
    obj))

(defun radius-of-sphere (sp)
  (let ((csg (send (cadr (car (send sp :csg))) :csg)))
    (cadr (car csg))))


(defun cylinder->marker-msg (cyl header 
				 &key ((:color col) (float-vector 1.0 0 0))
				      ((:alpha a) 1.0))
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::CYLINDER
                       :header header))
        (cent-coords (send cyl :copy-worldcoords))
        (height (height-of-cylinder cyl))
        (radius (radius-of-cylinder cyl)))
    (send cent-coords :translate 
          (float-vector 0 0 (/ height 2)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation 
                      (float-vector (* 2 radius) (* 2 radius) height)))
    (send msg :color (vector->rgba col a))
    msg))

(defun cube->marker-msg (cb header
			    &key ((:color col) (float-vector 1.0 0 0))
   			         ((:alpha a) 1.0))
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::CUBE
                       :header header))
        (cent-coords (send cb :copy-worldcoords))
        (x (x-of-cube cb))
        (y (y-of-cube cb))
        (z (z-of-cube cb)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation 
                      (float-vector x y z)))
    (send msg :color (vector->rgba col a))
    msg))

(defun sphere->marker-msg (sp header
			    &key ((:color col) (float-vector 1.0 0 0))
   			         ((:alpha a) 1.0))
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::SPHERE
                       :header header))
        (cent-coords (send sp :copy-worldcoords))
        (r (radius-of-sphere sp)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation 
                      (scale 2.0 (float-vector r r r))))
    (send msg :color (vector->rgba col a))
    msg))

(defun vector->rgba (cv &optional (alpha 1.0))
  (instance std_msgs::ColorRGBA :init
            :r (elt cv 0)
            :g (elt cv 1)
            :b (elt cv 2)
            :a alpha))

(defun test-pub ()
  (make-irtviewer)
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
       (ros::publish *pub-cylinder-topic* cyl-msg)
       (ros::publish *pub-cube-topic* cb-msg)
       (ros::publish *pub-sphere-topic* sp-msg)
       (send cyl :rotate (deg2rad 0.1) :x)
       (send cb :rotate (deg2rad 0.1) :x)
       (send *irtviewer* :draw-objects)
       (x::window-main-one)
       (unix:usleep 10000)
       ))))

(ros::advertise *pub-cylinder-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-cube-topic* visualization_msgs::Marker 5)
(ros::advertise *pub-sphere-topic* visualization_msgs::Marker 5)
(test-pub)
