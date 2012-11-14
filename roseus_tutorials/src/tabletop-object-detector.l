#!/usr/bin/env roseus

(defvar *transform-table* t)

(ros::load-ros-manifest "tabletop_object_detector")

(ros::roseus "roseus_tabletop_object_detector")

(ros::wait-for-service "/tabletop_segmentation")

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun tabletop-detect()
  (let ((req (instance tabletop_object_detector::TabletopSegmentationRequest :init))
        res ret convex-hull msg)
    ;; PR2
    (when nil ;;(substringp "http://pr1012" (unix::getenv "ROS_MASTER_URI"))
      (setq convex-hull (instance arm_navigation_msgs::Shape :init
                                  :type arm_navigation_msgs::Shape::*MESH*
                                  :triangles (list 0 1 2 3)
                                  :vertices (mapcar #'ros::pos->tf-point (list #f(300 -1000 700) #f(300 1000 700) #f(1200 1000 700) #f(1200 -1000 700) ))
                                  ))
      (ros::ros-info ";; tabletop detection with given table surface ~A" (mapcar #'ros::tf-point->pos (send convex-hull :vertices)))
      (setq req (instance tabletop_object_detector::TabletopSegmentationRequest :init
                          :table (instance tabletop_object_detector::Table
                                           :init
                                           :pose
                                           (ros::coords->tf-pose-stamped (make-cascoords) "/base_link")
                                           :convex_hull convex-hull)))
      (setq msg (instance visualization_msgs::Marker :init
                          :action visualization_msgs::Marker::*ADD*
                          :lifetime (ros::time 5)
                          :type visualization_msgs::Marker::*LINE_STRIP*
                          :color  (vector->rgba #f(0 1 1) 1)
                          :scale  (ros::pos->tf-translation #f(10 10 10))
                          :points (send convex-hull :vertices)
                          :header (instance std_msgs::header :init :frame_id "/base_link")
                          :pose   (ros::coords->tf-pose (make-coords))
                          :ns     "tabletop_node"
                          :id     0))
      (ros::publish "tabletop_segmentation_markers" msg))
    ;; /PR2

    ;;
    ;; this program only segment points 0.5[m] above the tabale, based on the following param
    ;; <param if="$(arg tabletop_segmentation_convert_to_base_link)" name="table_z_filter_min" value="0.01" />
    ;; <param if="$(arg tabletop_segmentation_convert_to_base_link)" name="table_z_filter_max" value="0.5" />

    ;;
    (setq res (ros::service-call "tabletop_segmentation" req))
    (unless res
      (ros::ros-warn ";; tabletop service failed")
      (return-from tabletop-detect nil))

    (cond
     ((= (send res :result) tabletop_object_detector::TabletopSegmentation::*SUCCESS*)
      (ros::ros-info ";; tabletop detection succeeded ~d" (send res :result))
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
      (ros::ros-warn ";; tabletop detection failed ~d" (send res :result))
      (setq ret nil)
      ))
    ret
    ))

(ros::advertise "tabletop_detection_marker_array" visualization_msgs::MarkerArray 5)
(ros::advertise "tabletop_segmentation_markers" visualization_msgs::Marker 5)

(ros::rate 3)
(while (ros::ok)
  (ros::spin-once)
  (x::window-main-one)
  (let ((plst (tabletop-detect)))
    (when plst
      (let ((msg-array (instance visualization_msgs::MarkerArray :init))
            clst blst hdr (cntr 0))
        (pprint plst)
        ;; centroid list
        (setq clst (mapcar #'(lambda (p) (send p :centroid)) plst))
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
                            (wireframe->marker-msg cb (get p :header) :id (incf cntr) :lifetime 5))) plst))

        (ros::publish "tabletop_detection_marker_array" msg-array))))
  (ros::sleep)
  )
