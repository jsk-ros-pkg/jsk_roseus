#!/usr/bin/env runeus

(load "roseus.l")
(load "eustf.l")

(roseus "test_publish_marker")

(load-ros-manifest "test_publish_marker")

(defvar *pub-cylinder-topic* "/cylinder_marker")


(defun cylinder->marker-msg (cyl header)
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::CYLINDER
                       :header header))
        (cent-coords (send cyl :copy-worldcoords))
        (height (elt (car (send cyl :csg)) 2))
        (radius (elt (car (send cyl :csg)) 1)))
    (send cent-coords :translate 
          (float-vector 0 0 (/ height 2)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation 
                      (float-vector (* 2 radius) (* 2 radius) height)))
    (send msg :color (vector->rgba (float-vector 1.0 0 0)))
    msg))

(defun vector->rgba (cv &optional (alpha 1.0))
  (instance std_msgs::ColorRGBA :init
            :r (elt cv 0)
            :g (elt cv 1)
            :b (elt cv 2)
            :a alpha))

(defun test-pub ()
  (make-irtviewer)
  (let ((c (make-cylinder 1000 2000)))
    (send c :translate (float-vector 0 0 -1000))
    (objects (list c))
    (do-until-key
     (let* ((header (instance roslib::header :init 
                              :stamp (ros::time-now)
                              :frame_id "/test_id"))
            (msg (cylinder->marker-msg c header)))
       (ros::publish *pub-cylinder-topic* msg)
       (send (make-coords) :draw-on :flush t :width 3 :color #f(1 0 0) :size 500)
       (send c :rotate (deg2rad 0.1) :x)
       (send *irtviewer* :draw-objects)
       (x::window-main-one)
       (unix:usleep 10000)
       ))))

(ros::advertise *pub-cylinder-topic* visualization_msgs::Marker 5)
(test-pub)
