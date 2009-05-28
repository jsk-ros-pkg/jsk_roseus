(load "roseus.l")
(load "festival.l")
(print 'sinit0)
(unless (boundp '*festival*) (sinit :host "hrp2018v" :user "hrpuser"))
(print 'sinit1)

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "robot_msgs")
(roseus-add-msgs "robot_actions")
(roseus-add-msgs "nav_robot_actions")
;;;

(setq ros::*debug* t)
;;;
;;;
(roseus "clear")
(ros::advertise "goal" robot_msgs::PoseStamped 0)
;;(defun test2 (a) (print (cons a "test")) 1)
;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

;;;
  (setq *move-base-state* nil)

(defun test (&optional (map-name :icra))
  (case map-name
	(:icra
	 (setq *goal-list* (list 
			    ;;(list 19.0 13.7 30.0);;x,y,theta[deg]
			    (list 19.2 15.8 0.9696 -0.2444);;x,y,q-z,q-w
			    (list 17.7 11.6 0.9848 -0.1735)
			    (list 11.0 13.8 0.6618 0.7496)
			    (list 18.8 16.6 0.045314 0.998972)
			    ))
	 )
	(:workshop-corridor
	 (setq *goal-list* (list 
			    ;;x,y,q-z,q-w
			    (list 36.298 9.850 0.964 -0.267);; home
			    (list 28.146 19.387 0.819 0.574);; manual door
			    (list 25.130 25.990 0.383 0.924);; toilet
			    (list 30.103 28.028 0.230 0.973);; subway
			    (list 32.548 23.382 -0.526 0.851);;auto door
			    ))
	 )
	)
  (setq *move-base-state* nil)
  (setq i 1)
  (setq state :stop pre-state nil)
  (do-until-key
   (unix:sleep 3)
   ;; goal check
   (if *move-base-state*
       (setq state (send (send *move-base-state* :status ) :value)))
   ;;byte UNDEFINED=0
   ;;byte SUCCESS=1
   ;;byte ABORTED=2
   ;;byte PREEMPTED=3
   ;;byte ACTIVE=4
   (setq goal 0)
   (when (memq state (list 1 2)) ;; SUCCESS=1
     (when (and (boundp '*festival*)
		(eq map-name :workshop-corridor))
       (case (position goal *goal-list* :test #'eq)
	     (0 ;; :workshop-corridor
	      (sproc "This is home.")
	      )
	     (2
	      (sproc "This is rest room.")
	      )
	     (3
	      (sproc "This is subway.")
	      )
	     (t
	      (sproc "This is via point.")
	      )
	     )
       )
     (setq goal (elt *goal-list* (mod (1- i) (length *goal-list*))))
     (format t ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;~%")
     (format t ";; set goal ~A~%" goal)
     (format t ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;~%")
     (setq msg (instance robot_msgs::PoseStamped :init))
     ;; set pose
     (send msg :pose :position :x (elt goal 0))
     (send msg :pose :position :y (elt goal 1))
     (send msg :pose :position :z 0)
     #|
     (setq rot-c (instance coordinates :init :rpy (list (deg2rad (elt goal 2)) 0 0)))
     (setq rot-q (matrix2quaternion (send rot-c :worldrot)))
     (send msg :pose :orientation :x (elt rot-q 0))
     (send msg :pose :orientation :y (elt rot-q 1))
     (send msg :pose :orientation :z (elt rot-q 2))
     (send msg :pose :orientation :w (elt rot-q 3))
     |#
     (send msg :pose :orientation :z (elt goal 2))
     (send msg :pose :orientation :w (elt goal 3))
     ;; set header
     (send msg :header :seq i)
     (setq ts (instance ros::rostime :init)
	   tim (unix::gettimeofday))
     (send ts :sec (elt tim 0))
     (send ts :nsec (* (elt tim 1) 1000))
     (send msg :header :stamp ts)
     (send msg :header :frame_id "map")
     ;; publish
     ;;(unix:sleep 1)
     (ros::publish "goal" msg)
     (incf i)
     ))))

;;(ros::subscribe "move_base_node/feedback" nav_robot_actions::MoveBaseState 
(ros::subscribe "move_base_node/feedback" nav_robot_actions::MoveBaseState 
		#'(lambda (msg) (setq *move-base-state* msg)))
(ros::init-ros-wakeup-worker-thread)
(format t "(test :icra)~%")
(format t "(test :workshop-corridor)~%")
;;(test)
;(unix:sleep 3)
;(roseus "shutdown")
;(exit)
