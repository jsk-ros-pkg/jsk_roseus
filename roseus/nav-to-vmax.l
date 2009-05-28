(progn
  (require :orbixif (format nil "~A/hrp2/corba/orbixif.l" (unix:getenv "CVSDIR")))
  (require :plugins (format nil "~A/hrp2/plugins/plugins.l" (unix:getenv "CVSDIR")))
  )

(load "roseus.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "robot_msgs")
;;;

(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
;;(defun test2 (a) (print (cons a "test")) 1)
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

;;;
(unless (boundp '*hrp2*) (hrp2v-simple))
;;(unless (boundp '*ci*) (eusorbix-init))
(load (format nil "~A/hrp2w/vmax/main/vmax-interface.l" (unix:getenv "CVSDIR")))
  (unless (boundp '*vi*) 
    (setq *vi* (instance vmax-interface :init :host "hrp2018w"))    
    )

(print 'connected)
;;;

(defun nav-to-vmax
  (msg)
  ;;(print msg)
  (print (list :vel (send (send msg :vel) :vx) (send (send msg :vel) :vy) (send (send msg :vel) :vz)))
  (print (list :ang_vel (send (send msg :ang_vel) :vx) (send (send msg :ang_vel) :vy) (send (send msg :ang_vel) :vz)))
  (setq vel (list (send (send msg :vel) :vx)
		  (send (send msg :vel) :vy)
		  (rad2deg (send (send msg :ang_vel) :vz))))
  (print vel)
  ;;(send* *ci* :go-velocity vel)
  )
;;(ros::subscribe "cmd_vel" robot_msgs::PoseDot #'nav-to-vmax)
;;(ros::subscribe "cmd_vel" robot_msgs::PoseDot #'nav-to-vmax)
(setq *nav-to-vmax* #'(lambda (msg)
			(let (vel)
			  (setq vel (float-vector
				     (send (send msg :vel) :vx)
				     (send (send msg :vel) :vy)
				     (rad2deg (send (send msg :ang_vel) :vz))))
			  ;;(setq vel (scale 0.75 vel))
			  (print vel)
			  ;;(send *ci* :go-velocity (elt vel 0) (elt vel 1) (elt vel 2))
			  (send *vi* :go-velocity (elt vel 0) (elt vel 1) (elt vel 2))
			  )))
(ros::subscribe "cmd_vel" robot_msgs::PoseDot *nav-to-vmax*)
(ros::init-ros-wakeup-worker-thread)
