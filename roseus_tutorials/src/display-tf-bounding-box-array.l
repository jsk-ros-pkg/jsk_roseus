#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(setq *input-bbox-array* (ros::get-param "~/input_bbox_array" "/HSI_color_filter/boxes"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "/base_footprint"))
(setq *camera-frame-id* (ros::get-param "~camera_frame_id" ":head_mount_kinect_rgb_optical_frame_lk"))
;;(setq *robot* (ros::get-param "~robot" *pr2*))

(defun bounding-box-array-cb (msg)
  (unless (send msg :boxes)
    (return-from bounding-box-array-cb))
  (let ((bounding-box-list (send msg :boxes)))
    (when bounding-box-list
      (send *robot* :angle-vector (send *ri* :potentio-vector))
      (send *tfl* :wait-for-transform *base-frame-id* (send msg :header :frame_id) (ros::time 0) 10)
      (mapcar
       #'(lambda (b)
           (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                  (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
                  ;; Convert geoometry_msgs::Pose of BoundingBox to euslisp coordinates [mm]
                  (cam->obj-coords (ros::tf-pose->coords (send b :pose)))
                  ;; Get camera coordinates [mm]
                  (base->cam-coords (send *tfl* :lookup-transform
                            *base-frame-id* (send b :header :frame_id)
                            (ros::time 0))))
                  ;;(base->cam-coords (send (send *robot* *camera-frame-id*) :copy-worldcoords)))
              ;; Get BoundingBox world coordinates [mm]
              (send bx :transform 
                    (send (send base->cam-coords :copy-worldcoords)
                          :transform cam->obj-coords))
              (objects (list bx base->cam-coords *robot*))
             bx))
       bounding-box-list)
      )))

(ros::roseus "display-tf-bounding-box")
(setq *tfl* (instance ros::transform-listener :init))
(unless (boundp '*irtviewer*) (make-irtviewer))

(ros::subscribe *input-bbox-array* jsk_recognition_msgs::BoundingBoxArray
                #'bounding-box-array-cb 1)
(do-until-key
 (x::window-main-one)
 (ros::spin-once)
 (ros::sleep)
 )

