#!/usr/bin/env roseus

(ros::load-ros-manifest "tabletop_object_detector")

(ros::roseus "roseus_tabletop_object_detector")

(ros::wait-for-service "/tabletop_segmentation")

(defun tabletop-detect()
  (let ((req (instance tabletop_object_detector::TabletopSegmentationRequest :init))
        res ret)
    (setq res (ros::service-call "tabletop_segmentation" req))
    (unless res
      (ros::ros-warn ";; tabletop service failed")
      (return-from tabletop-detect nil))
    (cond
     ((= (send res :result) tabletop_object_detector::TabletopSegmentation::*SUCCESS*)
      ;;(setq *res* res)
      (setq ret
            (mapcar #'(lambda (p)
                        (let ((r (make-eus-pointcloud-from-ros-msg1 p)))
                          (setf (get r :header) (send p :header))
                          r))
                    (send res :clusters))))
     (t
      (ros::ros-warn ";; tabletop detection failed ~d~%" (send res :result))
      (setq ret nil)
      ))
    ret
    ))

(ros::advertise "tabletop_detection_marker_array" visualization_msgs::MarkerArray 5)

(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (x::window-main-one)
  (let ((plst (tabletop-detect))
        clst blst hdr (cntr 0)
        (msg-array (instance visualization_msgs::MarkerArray :init)))

    ;; centroid list
    ;;(setq clst (mapcar #'(lambda (p) (send p :centroid)) plst))
    ;; bounding box list
    ;;(setq blst (mapcar #'(lambda (p) (send p :box)) plst))

    (send msg-array :markers
          (mapcar #'(lambda (p)
                      (let* ((c (send p :centroid))
                             (b (send p :box))
                             (cb
                              (apply #'make-cube (coerce (send b :diagonal) cons))))
                        (send cb :translate c)
                        (send cb :worldcoords)
                        (cube->marker-msg cb (get p :header) :id (incf cntr)))) plst))

    (ros::publish "tabletop_detection_marker_array" msg-array)
    )
  (ros::sleep)
  )
