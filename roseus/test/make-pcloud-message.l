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
                        :stamp (ros::time-now) :frame_id frame))

    (setq msg (instance sensor_msgs::PointCloud :init
                        :header hdr
                        :points points-list
                        :channels channels))
    msg
    ))

(defun make-3dpointcloud-from-msg (msg &optional (3dp))
  (unless 3dp (setq 3dp (instance 3dpointcloud :init)))
  (let* ((pts (send msg :points))
         (chl (find-if #'(lambda (c) (string= (send c :name) "rgb")) (send msg :channels)))
         (size (length pts))
         (mat (make-matrix size 3))
         (mat-ary (array-entity mat))
         col col-ary)
    (let ((ptr 0))
      (dolist (pt pts)
        (setf (elt mat-ary ptr) (* 1000 (send pt :x)))
        (setf (elt mat-ary (+ ptr 1)) (* 1000 (send pt :y)))
        (setf (elt mat-ary (+ ptr 2)) (* 1000 (send pt :z)))
        (incf ptr 3)))
    ;;
    (when chl
      (setq col (make-matrix size 3)
            col-ary (array-entity col))
      (let* ((val (send chl :values))
             (addr (+ (sys::address val) 8)))
        (dotimes (i size)
          (let* ((ptr (* i 4))
                 (cntr (* i 3))
                 (b (sys::peek (+ addr ptr) :byte))
                 (g (sys::peek (+ addr ptr 1) :byte))
                 (r (sys::peek (+ addr ptr 2) :byte)))
            (setf (elt col-ary cntr) (/ r 255.0))
            (setf (elt col-ary (+ cntr 1)) (/ g 255.0))
            (setf (elt col-ary (+ cntr 2)) (/ b 255.0))
            ))
        ))
    (send 3dp :matrix mat)
    (if col (send 3dp :carray col))
    3dp
    ))
