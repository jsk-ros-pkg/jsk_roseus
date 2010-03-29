;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_server.py
;;;
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")
(load "actionlib.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-actionlibs "actionlib_tutorials")
;;;

;;;
;;;
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(defclass ros::simple-action-server
  :super ros::object
  :slots (name-space action-spec status
                     action-goal-class action-result-call action-feedback-class
                     execute-cb goal goal-id seq-id))

(defmethod ros::simple-action-server
  (:execute-cb
   ()
   (let (msg status-list)
     (if (and goal-id execute-cb) (funcall execute-cb self goal))
     (setq msg (instance actionlib_msgs::GoalStatusArray :init))
     (when goal-id
       (send msg :status_list
             (list (instance actionlib_msgs::goalstatus :init
                             :goal_id goal-id
                             :status status))))
     (send msg :header :seq (incf seq-id))
     (send msg :header :stamp (ros::rostime))
     (ros::publish (format nil "~A/status" name-space) msg)
     ))
  (:goal-callback
   (msg)
   (ros::ros-info ";; Recieved Goal")
   (setq status actionlib_msgs::active)
   (setq goal msg)
   (setq goal-id (send msg :goal_id)))
  (:set-succeeded
   (msg)
   (ros::ros-info ";; Send Result")
   (setq status actionlib_msgs::succeeded)
   (send msg :status :goal_id  goal-id)
   (send msg :status :status status)
   (send msg :header :stamp (ros::rostime))
   (ros::publish (format nil "~A/result" name-space) msg)
   (setq goal-id nil goal nil)
   (setq status actionlib_msgs::pending)
   )
  ;;
  (:goal () goal)
  (:result
   (&rest args)
   (let (msg)
     (setq msg (instance action-result-class :init))
     (send* msg :result args)
     msg))
  ;;
  (:init
   (ns spec &key ((:execute-cb f)))
   (setq name-space ns)
   (setq action-spec (instance spec :init))
   (setq execute-cb f)
   (setq comm-state (instance actionlib-comm-state :init))
   ;;
   (setq seq-id 0)
   (setq status actionlib_msgs::pending)
   ;;
   (setq action-goal-class     (class (send action-spec :action_goal))
         action-result-class   (class (send action-spec :action_result))
         action-feedback-class (class (send action-spec :action_feedback)))
   (ros::advertise (format nil "~A/status" name-space) actionlib_msgs::GoalStatusArray)
   (ros::advertise (format nil "~A/result" name-space) action-result-class)
   (ros::advertise (format nil "~A/feedback" name-space) action-feedback-class)
   (ros::subscribe (format nil "~A/goal" name-space)
                   action-goal-class #'send self :goal-callback)
   (ros::subscribe (format nil "~A/cancel" name-space)
                   actionlib_msgs::GoalID #'send self :cancel-callback)
   ;; need getparam for status_frequence, status_list_timeout
   )
  ;;
  (:worker () (send self :execute-cb))
  )

(roseus "fibonacci")
(defun fibonacci-execute-cb (server goal)
  (let ((order (send goal :goal :order))
        (sequence (list 1 0)) msg)
    (dotimes (i (1- order))
      (push (+ (car sequence) (cadr sequence)) sequence))
    (setq msg (send server :result :sequence (nreverse sequence)))
    (send server :set-succeeded msg)
    ))

(setq s (instance ros::simple-action-server :init
                  "/fibonacci" actionlib_tutorials::FibonacciAction
                  :execute-cb #'fibonacci-execute-cb))

(ros::rate 10)
(do-until-key
 (send s :worker)
 (ros::spin-once)
 (ros::sleep)




