#!/usr/bin/env roseus

(defvar *transform-table* t)

(ros::load-ros-manifest "tabletop_object_detector")

(ros::roseus "roseus_tabletop_object_detector")

(ros::wait-for-service "/tabletop_segmentation")

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun tabletop-detect()
  (let ((req (instance tabletop_object_detector::TabletopSegmentationRequest :init))
        res ret)
    (setq res (ros::service-call "tabletop_segmentation" req))
    (unless res
      (ros::ros-warn ";; tabletop service failed")
      (return-from tabletop-detect nil))

    (cond
     ((= (send res :result) tabletop_object_detector::TabletopSegmentation::*SUCCESS*)
      (cond
       (*transform-table*
        ;;(setq *res* res)
        (let* ((tbl (send res :table))
               (pose (send tbl :pose))
               (cds (ros::tf-pose->coords (send pose :pose)))
               (icds (send cds :inverse-transformation))
               hdr)

          ;;(send cds :translate (float-vector (- (send tbl :x_min)) (- (send tbl :y_min)) 0))
          (send *tfb* :send-transform
                cds (send pose :header :frame_id) "/table"
                (send pose :header :stamp))

          (setq hdr (instance std_msgs::Header :init
                              :stamp (send pose :header :stamp)
                              :frame_id "/table"))
          (setq ret
                (mapcar #'(lambda (p)
                            (let ((r (make-eus-pointcloud-from-ros-msg1 p)))
                              (setf (get r :header) hdr)
                              (send r :transform-points icds)
                              r))
                        (send res :clusters)))
          ))
       (t
        (setq ret
              (mapcar #'(lambda (p)
                          (let ((r (make-eus-pointcloud-from-ros-msg1 p)))
                            (setf (get r :header) (send p :header))
                            r))
                      (send res :clusters)))))
      )
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
  (let ((plst (tabletop-detect)))
    (when plst
      (let ((msg-array (instance visualization_msgs::MarkerArray :init))
            clst blst hdr (cntr 0))

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

        (ros::publish "tabletop_detection_marker_array" msg-array))))
  (ros::sleep)
  )
