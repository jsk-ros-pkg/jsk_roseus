;; -*- mode: EusLisp; coding: utf-8-unix; -*-

(load (format nil "~A/prog/jvl/app/test-sr-capture.l" (unix:getenv "HOME")))
(init-cap "many-packs")

(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "geometry_msgs")
(roseus-add-msgs "sensor_msgs")

(defun proc-pcloud ()
  (proc-3dpos)
  (setq *3dp* (send *sr* :point-cloud))
  )

(defvar P_WIDTH 176)
(defvar P_HEIGHT 144)

(setq *points-num* (* P_WIDTH P_HEIGHT))

;;(defvar P_LENGTH 16)
;;(setq *points-num* (* P_LENGTH P_LENGTH))

(roseus "test_pcloud")
(ros::advertise "pcloud" sensor_msgs::PointCloud 5)
(ros::rate 0.5)

(let ((count 0)
      points channels
      channel-col hdr msg)

  (dotimes (i *points-num*)
    (push (instance geometry_msgs::Point32 :init) points))

  (while (ros::ok)
    (proc-pcloud)

    (setq channel-col (make-array *points-num* :initial-element 0.0 :element-type :float))

    ;; make-points
    (let* ((cntr 0)
           (carray (send *3dp* :carray))
           (parray (send *3dp* :matrix)))
      (dolist (p points)
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
                        :stamp (ros::rostime) :frame_id "/sensor_frame"))

    (setq msg (instance sensor_msgs::PointCloud :init
                        :header hdr
                        :points points
                        :channels channels))

    #|
  ;; debug messages                     ; ;
    (format t "~%HEADER ~A~%" (send hdr :serialization-length))
    (describe hdr)
    (pprint (send hdr :serialize))

    (format t "~%CHANNELS ~A~%" (send (car channels) :serialization-length))
    (describe channels)
    (pprint (send (car channels) :serialize))

;;  (format t "~%POINTS~%")             ; ;
;;  (pprint points)                     ; ;
    (format t "~%POINT ~A~%" (send (car points) :serialization-length))
    (describe (car points))
    (pprint (send (car points) :serialize))

    (format t "~%MESSAGE ~A~%" (send msg :serialization-length))
    (pprint (send msg :serialize))
    (describe msg)
    |#

    (ros::publish "pcloud" msg)
    (ros::sleep)
    (incf count)
    ;;(if (> count 100) (setq count 0))
    )
  )

(roseus "shutdown")
(exit)