#!/usr/bin/env roseus

(ros::roseus "eus_pointcloud")
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "sensor_msgs")

(setq *tfb* (instance ros::transform-broadcaster :init))

(setq *filename_a* "filename_a.l")
(setq *filename_b* "filename_b.l")

(setq *pos-array-y* (float-vector -100 0 100 -100 0 100 -100 0 100))
(setq *pos-array-z* (float-vector -100 -100 -100 0 0 0 100 100 100))

(defun make-eus-pointcloud1()
  (let (mat colors ary r g b pc)
    (setq mat (make-matrix 27 3))
    (setq colors (list))
    (setq ary (array-entity mat))
    (dotimes (i 81)
      (case (mod i 3)
        ;;x
        ( 0
          (case (/ i 27)
            (0
             (setf (elt ary i) 100)
             (setq r 1.0)
             )
            (1
             (setf (elt ary i) 0)
             (setq r 0.5)
             )
            (2
             (setf (elt ary i) -100)
             (setq r 0)
             )
            )
          )
        ;;y
        ( 1
          (setf (elt ary i) (elt *pos-array-y* (mod (/ i 3) 9)))
          (setq b (/ 1.0 (mod i 9)))
          )
        ;;z
        ( 2
          (setf (elt ary i) (elt *pos-array-z* (mod (/ i 3) 9)))
          (setq g ( - 1.0(/ 1.0 (mod i 9))))
          )
        )
      (if (equal (mod i 3) 2)
          (progn
;;            (ros::ros-info "~A ~A ~A ~%" (elt ary (- i 2)) (elt ary (- i 1)) (elt ary i))
            (push (float-vector r g b) colors)
            )
        )
      )
    (setq pc (instance pointcloud :init :points mat :colors colors))
    pc
    )
  )

(defun dump-eus-pointcloud()
  (dump-structure *filename_a* *pc*)
  (dump-loadable-structure *filename_b* *pc*)
  )

(defun load-eus-pointcloud ()
  (setq *pc_a* (with-open-file (f *filename_a*) (read f)))
  (ros::ros-info "~A : ~A" *filename_a* *pc_a*)
  (objects (list *pc_a*))

  (load *filename_b*)
  (ros::ros-info "~A : ~A" *filename_b* *pc*)
  (objects (list *pc*))
  )

(ros::advertise "pointcloud_transformed" sensor_msgs::PointCloud2 1)
(ros::advertise "pointcloud_not_transformed" sensor_msgs::PointCloud2 1)
(ros::advertise "pointcloud_wasnot_transformed" sensor_msgs::PointCloud2 1)

(setq *pc* (make-eus-pointcloud1))
(setq *pc1* (make-eus-pointcloud1))
(setq *pc2* (make-eus-pointcloud1))
(setq *pc3* (make-eus-pointcloud1))

(setq *pc1_transformed* (send *pc1* :transform (make-coords :pos #f(250 0 0))))
(setq *pc2_point_transformed* (send *pc2* :transform-points (make-coords :pos #f(-250 0 0)) :create t))
(setq *pc3_transformed* (send *pc3* :transform (make-coords :pos #f(0 -250 0))))
(send *pc3_transformed* :convert-to-world :create nil)

(objects (list *pc1_transformed* *pc2_point_transformed* *pc3_transformed*))

(ros::ros-info "")
(ros::ros-info ":create nil   will change the object")
(ros::ros-info ":create t     will return new object")

(setq *pc1_transformed_msg* (make-ros-msg-from-eus-pointcloud *pc1_transformed*))
(setq *pc2_point_transformed_msg* (make-ros-msg-from-eus-pointcloud *pc2_point_transformed*))
(setq *pc3_transformed_msg* (make-ros-msg-from-eus-pointcloud *pc3_transformed*))

(send *pc1_transformed_msg* :header :frame_id "/point_base")
(send *pc2_point_transformed_msg* :header :frame_id "/point_base")
(send *pc3_transformed_msg* :header :frame_id "/point_base")

(ros::ros-info "(dump-eus-pointcloud)")
(ros::ros-info "   this will dump eus-pointcloud (this time *pc*)")
(ros::ros-info "(load-eus-pointcloud)")
(ros::ros-info "   this will load eus-pointcloud which was dumped by (dump-eus-pointcloud) ")
(ros::ros-info "(main)")
(ros::ros-info "   this will start ros publish loop. Open rviz and subscribe PointCloud2")

(defun publish-pointcloud()
  (ros::publish "pointcloud_not_transformed" *pc1_transformed_msg*)
  (ros::publish "pointcloud_transformed" *pc2_point_transformed_msg*)
  (ros::publish "pointcloud_wasnot_transformed" *pc3_transformed_msg*)
  )

(defun main()
  (ros::rate 10)
  (while (ros::ok)
    (send *tfb* :send-transform (make-coords) "/world" "/point_base")
    (ros::spin-once)
    (publish-pointcloud)
    (x::window-main-one)
    (ros::sleep)
    )

