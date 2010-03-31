;; actionlib-base-node.l
;; implementation of actionlib which has the same interface
;; to the navigation stack
;;
;; currently only one status list is supported

;; SPEC of navigation stack
;; status:
;;   when moving -> status = 1, pending
;;   arrived at goal -> status status = 3, succeed
;; feedback:
;;   current position of robot
;; result:
;;   not used, but update when arriving at goal

;; we can run navigation stack
;;  $ roslaunch pr2_2dnav_gazebo pr2-armless-wg-fake_localization.launch
;;  $ roslaunch 2dnav_pr2 rviz_move_base.launch

;; ros setup
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")
(load "actionlib.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "geometry_msgs") ;move_base_msgs depends on geometry_msgs
(roseus-add-msgs "move_base_msgs")

;; hrp2 setup
(load (format nil "~A/hrp2/corba/orbixif.l" (unix::getenv "CVSDIR")))
(load (format nil "~A/hrp2/plugins/plugins.l" (unix::getenv "CVSDIR")))
;; global variables
(defvar *app* nil)

(defun navigation-set-status (server status)
  "temp function.
set the current status to `status'.
examples:
  (navigation-set-status server actionlib_msgs::pending)"
  (let ((msg (instance actionlib_msgs::GoalStatusArray :init)))
    ;; setup header of msg
    (send msg :header :stamp (ros::rostime))
    (send msg :header :seq (send server :next-seq-id))
    (send msg :status_list)
    (ros::publish (format nil "~a/status" (send server :name-space)) msg))
  )

;; Header header
;; actionlib_msgs/GoalID goal_id
;; MoveBaseGoal goal -> geometry_msgs/PoseStamped target_pose?
(defun navigation-execute-cb (server goal)
  "`goal' is an instance of move_base_msgs::move-base-goal, i think..."
  ;; not implemented now
  ;; this function is called everytime when clients set the new goal.
  ;; first, we need to set the `status' to PENDING
  (navigation-set-status server actionlib_msgs::pending)
  (let ((pose-stamped (send goal :goal)))
    
    ))

(defun init ()
  (ros::ros-info "actionlib-base-node is started now!")
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
