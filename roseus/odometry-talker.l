(load "roseus.l")
(load (format nil "~A/hrp2w/vmax/main/vmax-interface.l" (unix:getenv "CVSDIR")))

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
;;;

(setq ros::*debug* nil)
;;;
;;;
(roseus "clear")
(ros::advertise "odometry" roseus::Odometry 100)

(unless (boundp '*vi*)
  (setq *vi* (instance vmax-interface :init :host "hrp2018w")))

;;(do-until-key
(while t
  (if (select-stream (list *standard-input*) 0.0000001) (return nil))
 (setq xyr (send *vi* :vmax-worldpos))
 (setf (elt xyr 2) (deg2rad (elt xyr 2)))
 (setq msg (instance roseus::Odometry :init))
 (send msg :xyr xyr)
 (send msg :header :frame_id "odom")
 ;; odom is at the start point
 ;; base_link is center of robot
 (setq ts (instance ros::rostime :init)
       tim (unix::gettimeofday))
 (send ts :sec (elt tim 0))
 (send ts :nsec (* (elt tim 1) 1000))
 (send msg :header :stamp ts)
 (ros::publish "odometry" msg)
 (print (send msg :xyr))
 (unix:usleep 1000)
 )
;(roseus "shutdown")
;(exit)

