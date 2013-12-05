#!/usr/bin/env roseus
(ros::load-ros-manifest "dynamic_reconfigure")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "jsk_topic_tools")

(ros::roseus "roi_reconfigure_call")

(defun call-roi (width height x_off y_off &key (decimate_x 2) (decimate_y 2)
                       (service-name "roi_crop_decimate/set_parameters"))
  (ros::ros-info "ROI: ~D ~D ~D ~D" width height x_off y_off)
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (send req :config
          :ints
          (list
           (instance dynamic_reconfigure::IntParameter :init :name "decimation_x" :value decimate_x)
           (instance dynamic_reconfigure::IntParameter :init :name "decimation_y" :value decimate_y)
           (instance dynamic_reconfigure::IntParameter :init :name "x_offset" :value x_off)
           (instance dynamic_reconfigure::IntParameter :init :name "y_offset" :value y_off)
           (instance dynamic_reconfigure::IntParameter :init :name "width" :value width)
           (instance dynamic_reconfigure::IntParameter :init :name "height" :value height)
           )
          )
    (ros::service-call service-name req)
    (setq *update* (ros::time-now))
    ))

(defun callback-angle (msg)
  (let ((pts (send msg :polygon :points))
        lst)
    (setq lst
          (mapcar #'(lambda (x) (list (round (send x :x)) (round (send x :y)))) pts))
    (let ((x_st (caar lst))
          (y_st (cadr (car lst)))
          (x_ed (car (cadr lst)))
          (y_ed (cadr (cadr lst))))
      (ros::ros-info "rect: ~D ~D ~D ~D" x_st y_st x_ed y_ed)
      (if (> x_st x_ed) (let ((tmp x_st)) (setq x_st x_ed x_ed tmp)))
      (if (> y_st y_ed) (let ((tmp y_st)) (setq y_st y_ed y_ed tmp)))
      (call-roi (- x_ed x_st) (- y_ed y_st) x_st y_st)
      )
    (setq *update* (ros::time-now))
    ))

(setq *default-roi-width* 320)
(setq *default-roi-height* 240)
(defun callback-point (msg)
  (let* ((pt (send msg :point))
         (x (round (send pt :x)))
         (y (round (send pt :y)))
         (st_x (- x (/ *default-roi-width* 2)))
         (st_y (- y (/ *default-roi-height* 2)))
         )
    (if (< st_x 0) (setq st_x 0))
    (if (< st_y 0) (setq st_y 0))

    (call-roi *default-roi-width* *default-roi-height* st_x st_y)
    ))

(ros::subscribe "/image_rect_color/screenrectangle" geometry_msgs::PolygonStamped
                #'callback-angle)

(ros::subscribe "/image_rect_color/screenpoint" geometry_msgs::PointStamped
                #'callback-point)

(setq *update* nil)
(ros::rate 20)
(do-until-key
 (ros::spin-once)
 (ros::sleep)
 (when (and *update*
            (> (- (send (ros::time-now) :to-sec)
                  (send *update* :to-sec)) 0.4))
   (let ((req (instance jsk_topic_tools::UpdateRequest
                        :init :topic_name "/roi_camera_output/image_raw")))
     (ros::service-call
      "/roi_topic_buffer_server/update" req)
     (setq *update* nil)
     )
   #|
   (let ((msg (instance std_msgs::String :init
                        :data "/roi_camera_output/image_raw")))
     (ros::publish "/roi_topic_buffer_server/update" msg)
     (setq *update* nil)
     )
   |#
   )
 )
