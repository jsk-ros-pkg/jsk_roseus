#!/usr/bin/env roseus

(defvar *result-topic* "/tabletop/ObjectDetection")
(defvar *visualize-topic* "tabletop_detection_marker_array")

(ros::load-ros-manifest "tabletop_object_detector")
(ros::roseus-add-msgs "posedetection_msgs")

(ros::roseus "roseus_tabletop_object_detector")

(ros::defrosparam *transform-table* "~transform_table" t)

(ros::advertise *result-topic* posedetection_msgs::ObjectDetection)
(ros::wait-for-service "/tabletop_segmentation")

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun tabletop-detect()
  (let ((req (instance tabletop_object_detector::TabletopSegmentationRequest :init))
        res ret msg)
    ;;
    ;; this program only segment points 0.5[m] above the tabale, based on the following param
    ;; <param if="$(arg tabletop_segmentation_convert_to_base_link)" name="table_z_filter_min" value="0.01" />
    ;; <param if="$(arg tabletop_segmentation_convert_to_base_link)" name="table_z_filter_max" value="0.5" />

    ;; call service
    (setq res (ros::service-call "tabletop_segmentation" req))
    (unless res
      (ros::ros-warn ";; tabletop service failed")
      (return-from tabletop-detect nil))

    (cond
     ((= (send res :result)
         tabletop_object_detector::TabletopSegmentation::*SUCCESS*)
      (ros::ros-info ";; tabletop detection succeeded ~d" (send res :result))
      (cond
       (*transform-table*
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

(ros::advertise *visualize-topic* visualization_msgs::MarkerArray 5)
;;(ros::advertise "tabletop_segmentation_markers" visualization_msgs::Marker 5)

;;
;; 処理をしているところ (main loop)
;;
(ros::rate 3)
(while (ros::ok)
  (ros::spin-once)
  (if x::*display* (x::window-main-one))
  ;;; *result-topic*
  (when (> (ros::get-num-subscribers *result-topic*) 0)
  (let ((plst (tabletop-detect)))
    (when plst
      (sort plst #'(lambda (a b) (<= (elt (send a :centroid) 0) (elt (send b :centroid) 0))))
      (let ((msg-array (instance visualization_msgs::MarkerArray :init))
            clst blst hdr (cntr 0))
        ;; PointCloudの表示
        ;;(pprint plst)
        ;; centroid list
        ;;(setq clst (mapcar #'(lambda (p) (send p :centroid)) plst))
        ;; 重心の表示
        ;;(pprint clst)
        ;; bounding box list
        ;;(setq blst (mapcar #'(lambda (p) (send p :box)) plst))
        ;; bounding boxの表示
        ;;(pprint blst)

        ;; /ObjectDetectionをpublish
        (let ((res (instance posedetection_msgs::ObjectDetection :init))
              (cnt 0) lst)
          (dolist (p plst)
            (let* ((c (send p :centroid))
                   (bb (send p :box))
                   (o6dp (instance posedetection_msgs::Object6DPose :init))
                   (pos (send bb :center)))
              (send o6dp :pose (ros::coords->tf-pose (make-coords :pos pos)))
              (send o6dp :type (format nil "tabletop~D" cnt))
              (incf cnt)
              (push o6dp lst)
              ))
          (send res :header (get (car plst) :header))
          (send res :objects lst)
          (ros::publish *result-topic* res))

        (send msg-array :markers
              (mapcar #'(lambda (p)
                          (let* ((c (send p :centroid))
                                 (b (send p :box))
                                 (cb
                                  (apply #'make-cube (coerce (send b :diagonal) cons))))
                            (send cb :translate c)
                            (send cb :worldcoords)
                            ;;(print (send cb :vertices))
                            (wireframe->marker-msg cb (get p :header) :id (incf cntr) :lifetime 5))) plst))

        (ros::publish *visualize-topic* msg-array))))
  ) ;; / when
  (ros::sleep)
  )
