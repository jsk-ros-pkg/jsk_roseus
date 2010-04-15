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

;; how to use:
;; NOP, NOP, NOP... 

;; we can run navigation stack
;;  $ ROBOT=sim roslaunch pr2_2dnav_gazebo pr2-armless-wg-fake_localization.launch
;;  $ roslaunch 2dnav_pr2 rviz_move_base.launch

;; ros setup
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")
(load "actionlib.l")

;; require 
(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "geometry_msgs") ;move_base_msgs depends on geometry_msgs
(roseus-add-msgs "move_base_msgs")
(roseus "move_base")

(load "geometry-util.l")

;; hrp2 setup
(load (format nil "~A/hrp2/corba/orbixif.l" (unix::getenv "CVSDIR")))
(load (format nil "~A/hrp2/plugins/plugins.l" (unix::getenv "CVSDIR")))

;; parameters for this script
(defvar *real* nil)

;; global variables
(defvar *app* nil)

;; util, i think these function can be used in ros-eus application in common

(defun navigation-set-status (server status &optional (text nil))
  "temp function.
set the current status to `status'.
examples:
  (navigation-set-status server actionlib_msgs::pending)"
  (let ((msg (instance actionlib_msgs::GoalStatusArray :init)))
    ;; setup header of msg
    (send msg :header :stamp (ros::time-now))
    (send msg :header :seq (send server :next-seq-id))
    ;;NB: need to append status_list
    (send msg :status_list (list (instance actionlib_msgs::goalstatus :init
                                           :status status
                                           :text (or text (goal-status-to-string status)))))
    (ros::publish (ros::append-name-space (send server :name-space) "status") msg)))

;; Header header
;; actionlib_msgs/GoalID goal_id
;; MoveBaseGoal goal -> geometry_msgs/PoseStamped target_pose?
(defun navigation-execute-cb (server goal)
  "`goal' is an instance of move_base_msgs::move-base-goal, i think..."
  (ros::ros-info "navigation-execute-cb is called")
  ;; this function is called everytime when clients set the new goal.
  ;; first, we need to set the `status' to PENDING
  (navigation-set-status server actionlib_msgs::pending)
  (navigation-go-pos goal)     ;does not wait interpolation
  (navigation-update-feedback-until-arrival server)
  (let ((msg (send server :result)))
    ;; NB: ??
    (send server :set-succeeded msg))
  t)

(defun navigation-go-pos (msg)
  "`goal' is an instance of MoveBaseActionGoal"
  ;; not implemented now
  (ros::ros-info "navigation-go-pos is called")
  (let* ((goal (send msg :goal))
         (pose (send (send goal :target_pose) :pose))
         (coords (geometry-msgs-pose->coords pose)))
    (let ((x (x (send coords :worldpos)))
          (y (y (send coords :worldpos)))
          (r (caar (send coords :rpy-angle))))
      (ros::ros-info (format nil "go toward (~A ~A ~A) -- relative" x y r))
      (when *real*
        (send *ci* :go-pos-no-wait (* 0.001 x) (* 0.001 y) (rad2deg r)))))
  )

(defun navigation-update-feedback-until-arrival (server)
  (ros::ros-info "navigation-update-feedback-until-arrival is called")
  (when *real*
    (while (not (send *ci* :walkingp)) (unix:usleep 100000)) ;wait until walkingp is true
    (while (send *ci* :walkingp)                             ;update feedback until walkingp is nil
      (let ((current-coords (send *ci* :state :reference-worldcoords)))
        (let ((pose (coords->geometry-msgs-pose current-coords)))
          ;; NB: need to add goal status
          (let ((msg (instance move_base_msgs::MoveBaseActionFeedback :init 
                               :feedback 
                               (instance move_base_msgs::MoveBaseFeedback :init
                                         :base_position (instance geometry_msgs::PoseStamped :init :pose pose)))))
            (ros::publish (ros::append-name-space (send server :name-space) "feedback") msg)))))))

(defun init-real ()
  (unless (boundp '*hrp2*)
    ;; make a robot model for crating *ci*
    (ros::ros-info (format nil "ROBOT=~A" (unix:getenv "ROBOT")))
    (case (intern (unix:getenv "ROBOT"))
          (hrp2jsk
           (hrp2jsk-simple-detail))
          (hrp2jsknt
           (hrp2jsknt-simple-detail))
          (hrp2w
           (hrp2w-simple-detail))))
  (ros::ros-info (format nil "connecting hrp2 No. ~A" (unix:getenv "HRP2NO")))
  (eusorbix-init)
  (ros::ros-info (format nil "connected to ~A" (send *ci* :host)))
  )

(defun init ()
  (when *real* (init-real))
  (ros::ros-info "actionlib-base-node is started now!")
  (setq *app* (instance ros::simple-action-server :init
                        "/move_base"                   ;namespace
                        move_base_msgs::MoveBaseAction      ;spec
                        :execute-cb #'navigation-execute-cb)) ;callback
  (ros::rate 1)                         ;setup rate
  )

(defun main ()
  (do-until-key
   (send *app* :worker)
   (ros::spin-once)
   (ros::sleep)))

(init)
(main)

;; (load "actionlib-base-node.l")
