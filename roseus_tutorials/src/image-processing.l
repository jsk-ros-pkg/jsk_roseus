#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")

;; vision callback
(defun cb (msg)
  (let (img width height)
    (setq width (send msg :width)
          height (send msg :height))
    (ros::ros-info "w: ~A, h: ~A, encoding: ~A" width height (send msg :encoding))
    (setq img (instance color-image24 :init width height (send msg :data)))
    ;; debug
    ;; (write-image-file "image.png" img)
    ;; show original image
    (if (not (boundp '*v-color*))
        (setq *v-color* (instance x::xwindow :create :width width :height height)))
    (swap-rgb img)
    (send *v-color* :putimage (img . entity))
    (send *v-color* :flush)

    ;; show gray image
    (if (not (boundp '*v-gray*))
        (setq *v-gray* (instance x::xwindow :create :width width :height height)))
    (setq gray-image (send img :monochromize))
    (send *v-gray* :putimage ((send gray-image :to24). entity)) ;; to24 for x::window????
    (send *v-gray* :flush)

    ;; function not found
    ;; (send gray-image :edge1)
    ))

;; init roseus
(ros::roseus "image-processing")
(ros::subscribe "image" sensor_msgs::Image #'cb))

;; main loop
(warning-message 2 "start main loop~%")
(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  ;(x::window-main-one)
  )
(ros::exit)
