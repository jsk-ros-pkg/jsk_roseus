;; actionlib-base-node.l
;; implementation of actionlib which has the same interface
;; to the navigation stack
;;

;; ros setup
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")
(load "actionlib.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "geometry_msgs")       ;move_base_msgs depends on geometry_msgs
(roseus-add-msgs "move_base_msgs")

;; hrp2 setup
(load (format nil "~A/hrp2/corba/orbixif.l" (unix::getenv "CVSDIR")))
(load (format nil "~A/hrp2/plugins/plugins.l" (unix::getenv "CVSDIR")))
;; global variables
(defvar *app* nil)



;; navigation stack's goal => geometry_msgs/PoseStamped target_pose
(defun navigation-execute-cb (server goal)
  ;; not implemented now
  )

(defun init ()
  (setq *app* (instance ros::simple-action-server :init
                        "/move_base_msgs"                   ;namespace
                        move_base_msgs::MoveBaseAction      ;spec
                        :execute-cb #'navigation-execute-cb ;callback
                        ))
  (ros::rate 1)                         ;setup rate
  )

(defun main ()
  (do-until-key
   (send *app* :worker)
   (ros::spin-once)
   (ros::sleep)))

;;(init)
;;(main)
