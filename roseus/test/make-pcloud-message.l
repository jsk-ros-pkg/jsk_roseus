(defun make-pointcloud-message (3dp points-list &optional (frame "/sensor_frame"))
  (let (channels
        channel-col hdr msg)

    (setq channel-col (make-array (send 3dp :size) :initial-element 0.0 :element-type :float))

    ;; make-points
    (let* ((cntr 0)
           (carray (send 3dp :carray))
           (parray (send 3dp :matrix)))
      (dolist (p points-list)
        (let* ((pt (matrix-row parray cntr))
               (cl (matrix-row carray cntr))
               (ival (+ (* #x10000 (floor (* 255 (x cl))))
                        (* #x100   (floor (* 255 (y cl))))
                                   (floor (* 255 (z cl)))))
               )
          (send p :x (/ (x pt) 1000.0))
          (send p :y (/ (y pt) 1000.0))
          (send p :z (/ (z pt) 1000.0))
          (sys::poke ival (+ (sys::address channel-col) 8 (* cntr 4)) :integer)

          (incf cntr)
          ))
      )

    ;; /make-points
    (setq channels (list (instance sensor_msgs::ChannelFloat32 :init
                                   :name "rgb" :values channel-col)))

    (setq hdr (instance roslib::header :init
                        :stamp (ros::rostime) :frame_id frame))

    (setq msg (instance sensor_msgs::PointCloud :init
                        :header hdr
                        :points points-list
                        :channels channels))
    msg
    ))
