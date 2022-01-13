#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")
(defvar *base-frame-id* "/base_footprint")
(defvar *camera-frame-id* "/head_mount_kinect_rgb_optical_frame")

(defun bounding-box-array-cb (msg)
  (let ((bounding-box-list (send msg :boxes)))
    (when bounding-box-list
      (send *irtviewer* :draw-objects :flush nil)
      (mapcar
       #'(lambda (b)
           (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                  (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
                  ;; Convert geoometry_msgs::Pose of BoundingBox to euslisp coordinates [mm]
                  (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
                  ;; Get camera coordinates [mm]
                  (cam-coords (send *tfl* :lookup-transform
                            *base-frame-id* (send msg :header :frame_id)
                            (ros::time 0))))
              ;; Get BoundingBox world coordinates [mm]
              (send bx :transform 
                    (send (send cam-coords :copy-worldcoords) 
                          :transform cam->obj-coords))
              (send bx :draw-on :flush nil :color #f(1 0 0))
             bx))
       bounding-box-list)
      (send *irtviewer* :viewer :viewsurface :flush)
      )))

(ros::roseus "bounding_box_array_subscriber")
(setq *tfl* (instance ros::transform-listener :init))
(unless (boundp '*irtviewer*) (make-irtviewer))

(ros::subscribe "~input_box" jsk_recognition_msgs::BoundingBoxArray
                #'bounding-box-array-cb 1)
(do-until-key
 (x::window-main-one)
 (ros::spin-once)
 (ros::sleep)
 )

