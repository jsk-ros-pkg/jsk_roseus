#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")


(defun bounding-box-array-cb (msg)
  (let ((bounding-box-list (send msg :boxes)))
    (when bounding-box-list
      (send *irtviewer* :draw-objects :flush nil)
      (mapcar
       #'(lambda (b)
           (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                  (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
                  (cam->obj-coords (ros::tf-pose->coords (send b :pose))))
             (send bx :newcoords cam->obj-coords)
             (send bx :worldcoords)
             (send bx :draw-on :flush nil :color #f(1 0 0))
             bx))
       bounding-box-list)
      (send *irtviewer* :viewer :viewsurface :flush)
      )))


(ros::roseus "bounding_box_array_subscriber")
(unless (boundp '*irtviewer*) (make-irtviewer))

(ros::subscribe "~input_boxes" jsk_recognition_msgs::BoundingBoxArray
                #'bounding-box-array-cb 1)

(do-until-key
 (x::window-main-one)
 (ros::spin-once)
 (ros::sleep)
 )
