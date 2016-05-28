#!/usr/bin/env roseus

(defvar *result-topic* "/tabletop/ObjectDetection")
(defvar *visualize-topic* "tabletop_detection_marker_array")
(defvar *bounding-box-array-topic* "tabletop_object_bounding_box_array")

(ros::load-ros-manifest "tabletop_object_detector")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(ros::roseus "roseus_tabletop_object_detector")

(ros::defrosparam *transform-table* "~transform_table" t)
(ros::defrosparam *always-publish* "~always_publish" nil)
(ros::defrosparam *publish-tf* "~publish_tf" t)

(ros::advertise *result-topic* posedetection_msgs::ObjectDetection 5)
(ros::advertise *visualize-topic* visualization_msgs::MarkerArray 5)
(ros::advertise *bounding-box-array-topic* jsk_recognition_msgs::BoundingBoxArray 5)

(setq *tfb* (instance ros::transform-broadcaster :init))

(defun tabletop-detect()
  ;; this program only segment points 0.5[m] above the tabale, based on the following param
  ;; <param if="$(arg tabletop_segmentation_convert_to_base_link)" name="table_z_filter_min" value="0.01" />
  ;; <param if="$(arg tabletop_segmentation_convert_to_base_link)" name="table_z_filter_max" value="0.5" />
  (let (res)
    ;; call service
    (setq res (ros::service-call "tabletop_segmentation"
               (instance tabletop_object_detector::TabletopSegmentationRequest :init)))
    (unless res
      (ros::ros-warn ";; tabletop service failed")
      (return-from tabletop-detect nil))

    (cond
     ((= (send res :result)
         tabletop_object_detector::TabletopSegmentationResponse::*SUCCESS*)
      (ros::ros-info ";; tabletop detection succeeded ~d" (send res :result))
      (cond
       (*transform-table*
        (let* ((tbl (send res :table))
               (pose (send tbl :pose))
               (cds (ros::tf-pose->coords (send pose :pose)))
               (icds (send cds :inverse-transformation))
               hdr)
          (send *tfb* :send-transform
                cds (send pose :header :frame_id) "/table"
                (send pose :header :stamp))
          (setq hdr (instance std_msgs::Header :init
                              :stamp (send pose :header :stamp)
                              :frame_id "/table"))
          (return-from tabletop-detect
                (mapcar #'(lambda (p)
                            (let ((r (make-eus-pointcloud-from-ros-msg1 p)))
                              (setf (get r :header) hdr)
                              (send r :transform-points icds)
                              r))
                        (send res :clusters)))))
       (t
        (return-from tabletop-detect
              (mapcar #'(lambda (p)
                          (let ((r (make-eus-pointcloud-from-ros-msg1 p)))
                            (setf (get r :header) (send p :header))
                            r))
                      (send res :clusters)))))
      )
     (t
      (ros::ros-warn ";; tabletop detection failed ~d" (send res :result))
      (return-from tabletop-detect nil)))))

(defun make-bounding-box-marker-array-msg (pc-lst)
  (let* ((cntr 0)
         (markers (mapcar #'(lambda (p)
                              (let* ((c (send p :centroid))
                                     (b (send p :box))
                                     (cb
                                      (apply #'make-cube (coerce (send b :diagonal) cons))))
                                (send cb :translate c)
                                (send cb :worldcoords)
                                (wireframe->marker-msg cb (get p :header)
                                                       :id (inc cntr)
                                                       :lifetime 5))) pc-lst)))
    (instance visualization_msgs::MarkerArray :init :markers markers)))

(defun float-vector->vector3 (fvec)
  (instance geometry_msgs::Vector3 :init
            :x (/ (aref fvec 0) 1000.0)
            :y (/ (aref fvec 1) 1000.0)
            :z (/ (aref fvec 2) 1000.0)))

(defun make-bounding-box-array-msg (pc-lst)
  (let* ((cntr 0)
         (boxes (mapcar #'(lambda (p)
                            (instance jsk_recognition_msgs::BoundingBox :init
                                      :header (get p :header)
                                      :pose (ros::coords->tf-pose
                                             (make-coords :pos (send p :centroid)))
                                      :dimensions
                                      (float-vector->vector3
                                       (scale 1.0 (send (send p :box) :diagonal)))))
                        pc-lst)))
    (instance jsk_recognition_msgs::BoundingBoxArray :init
              :header (get (car pc-lst) :header)
              :boxes boxes)))

(defun make-object-detection-msg (pc-lst)
  (let* ((cntr 0)
         (objs (mapcar #'(lambda (p)
                           (instance posedetection_msgs::Object6DPose :init
                                     :pose (ros::coords->tf-pose
                                            (make-coords :pos (send (send p :box) :center)))
                                     :type (format nil "tabletop_object~D" (inc cntr))))
                       pc-lst)))
    (instance posedetection_msgs::ObjectDetection :init
              :header (get (car pc-lst) :header)
              :objects objs)))

(defun publish-objects-tf (pc-lst)
  (let ((cntr 0))
    (dolist (p pc-lst)
      (send *tfb* :send-transform
            (make-coords :pos (send (send p :box) :center))
            (send (get p :header) :frame_id) (format nil "tabletop_object~D" (inc cntr))
            (send (get p :header) :stamp)))))

(defun check-subscriber ()
  (or
   (> (ros::get-num-subscribers *result-topic*) 0)
   (> (ros::get-num-subscribers *visualize-topic*) 0)
   (> (ros::get-num-subscribers *bounding-box-array-topic*) 0)))

(defun main ()
  (ros::rate 10)
  (ros::wait-for-service "/tabletop_segmentation")
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    (if x::*display* (x::window-main-one))
    (when (or *always-publish* (check-subscriber))
      (let ((plst (tabletop-detect)))
        (when plst
          (sort plst #'(lambda (a b) (<= (elt (send a :centroid) 0) (elt (send b :centroid) 0))))
          (when *publish-tf* (publish-objects-tf plst))
          (ros::publish *result-topic* (make-object-detection-msg plst))
          (ros::publish *visualize-topic* (make-bounding-box-marker-array-msg plst))
          (ros::publish *bounding-box-array-topic* (make-bounding-box-array-msg plst)))))))
(main)
