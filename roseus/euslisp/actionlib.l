(in-package "ROS")

(if (not (find-package "ACTIONLIB_MSGS")) (ros::roseus-add-msgs "actionlib_msgs"))

;;;; actionlib
(setq ros::*simple-goal-state-pending* 0
      ros::*simple-goal-state-active*  1
      ros::*simple-goal-state-done*    2)

(load "actionlib-commstate.l")

(defclass ros::simple-action-client
  :super ros::object
  :slots (name-space action-spec simple-state comm-state
                     action-goal-class action-result-class action-feedback-class
                     action-done-cb action-active-cb action-feedback-cb
                     goal_id))

(defmethod ros::simple-action-client
  (:goal-status-cb
   (msg)
   (ros::ros-debug ";; status ~A~%" msg)
   (let ((m (send comm-state :find-status-by-goal-id msg)))
     (when m
       (let ((com-state (send comm-state :update-status msg))
             (error-msg
              (format nil "Received comm state ~A when in simple state ~A"
                      comm-state simple-state)))
         (case
             com-state
           ('ros::*comm-state-active*
             (cond ((= simple-state ros::*simple-goal-state-pending*)
                    (setq simple-state ros::*simple-goal-state-active*))
                   ((=  simple-state ros::*simple-goal-state-done*)
                    (ros::ros-error error-msg))))
           ('ros::*comm-state-recalling*
             (cond ((not (= simple-state ros::*simple-goal-state-pending*))
		    (ros::ros-error error-msg))))
           ('ros::*comm-state-preempting*
             (cond ((= simple-state ros::*simple-goal-state-pending*)
                    (setq simple-state ros::*simple-goal-state-active*))
                   ((=  simple-state ros::*simple-goal-state-done*)
                    (ros::ros-error error-msg))))
           ('ros::*comm-state-done*
             (cond ((member simple-state
                            (list ros::*simple-goal-state-pending*
                                  ros::*simple-goal-state-active*))
                    (setq simple-state ros::*simple-goal-state-done*))
                   )))
         ))))
  (:action-result-cb
   (msg)
   (let ()
     (ros::ros-debug ";; reuslt ~A~%" msg)
     (send comm-state :update-result msg)
     ))
  (:action-feedback-cb
   (msg)
   (ros::ros-debug ";; feedback ~A~%" msg)
   (if action-feedback-cb (funcall action-feedback-cb msg)))
  ;;
  (:make-goal-instance
   (&rest args)
   (let (msg)
     (setq msg (instance action-goal-class :init))
     (send* msg :goal args)
     msg))
  ;;
  (:init
   (ns spec)
   (setq name-space ns)
   (setq action-spec (instance spec :init))
   (setq comm-state (instance actionlib-comm-state :init))
   (setq action-goal-class     (class (send action-spec :action_goal))
         action-result-class   (class (send action-spec :action_result))
         action-feedback-class (class (send action-spec :action_feedback)))
   (ros::advertise (format nil "~A/goal" name-space) action-goal-class)
   (ros::advertise (format nil "~A/cancel" name-space) actionlib_msgs::GoalID)
   (ros::subscribe (format nil "~A/status" name-space)
                   actionlib_msgs::GoalStatusArray #'send self :goal-status-cb)
   (ros::subscribe (format nil "~A/result" name-space)
                   action-result-class #'send self :action-result-cb)
   (ros::subscribe (format nil "~A/feedback" name-space)
                   action-feedback-class #'send self :action-feedback-cb)
   (setq simple-state ros::*simple-goal-state-done*)
   self)
  (:wait-for-server
   (&optional (timeout nil))
   (let ((goal-topic (format nil "~A/goal" name-space))
         (cancel-topic (format nil "~A/cancel" name-space))
         (feedback-topic (format nil "~A/feedback" name-space))
         (result-topic (format nil "~A/result" name-space))
	 (count 0))
     (ros::rate 10)
     (while (or (= (ros::get-num-subscribers goal-topic) 0)
                (= (ros::get-num-subscribers cancel-topic) 0)
                (= (ros::get-num-publishers feedback-topic) 0)
                (= (ros::get-num-publishers result-topic) 0))
       (ros::ros-debug ";; num subscribers ~A ~d, ~A ~d"
		       goal-topic (ros::get-num-subscribers goal-topic)
		       cancel-topic (ros::get-num-subscribers cancel-topic))
       (ros::spin-once)
       (ros::sleep)
       (when (and timeout (>= count (* timeout 10)))
         (ros::ros-warn "action server(~A) is not found" name-space)
         (ros::ros-warn " goal=~d, cancel=~d, feedback=~d, result=~d"
                        (ros::get-num-subscribers goal-topic)
                        (ros::get-num-subscribers cancel-topic)
                        (ros::get-num-publishers feedback-topic)
                        (ros::get-num-publishers result-topic))
         (return-from :wait-for-server nil))
       (incf count))
     (ros::sleep 1)
     t))
  (:send-goal
   (goal &key done-cb active-cb feedback-cb)
   (let ((now (ros::time-now)))
     (setq goal_id (instance actionlib_msgs::GoalID :init
                             :id (format nil "~a" (send now :sec-nsec))))
     (send goal :goal_id goal_id)
     (setq simple-state ros::*simple-goal-state-pending*)
     (setq comm-state (instance actionlib-comm-state :init :action-goal goal))
     (if done-cb (setq action-done-cb done-cb))
     (if active-cb (setq action-active-cb active-cb))
     (if feedback-cb (setq action-feedback-cb feedback-cb))
     (ros::publish (format nil "~A/goal" name-space) goal)))
  (:send-goal-and-wait
   (goal &key (timeout 0))
   (send self :send-goal goal)
   (unless (send self :wait-for-result :timeout timeout)
     (send self :cancel-goal)
     (send self :wait-for-result :timeout timeout)
     ))
  (:wait-for-result
   (&key (timeout 0)) ;; sec
   (let ((start (ros::time-now)))
     (ros::rate 10)
     (while (ros::ok)
       (if (= simple-state ros::*simple-goal-state-done*)
	   (return))
       (if (> timeout 0)
	   (let* ((tm (ros::time- (ros::time-now) start)))
	     (if (> (send tm :to-sec) timeout) (return))))
       (ros::spin-once)
       (ros::sleep))
     (eq (send self :get-state) actionlib_msgs::succeeded)))
  (:get-result () (send (send comm-state :latest-result) :result))
  (:get-state ()
    (let (state)
      (setq state (send (send comm-state :latest-goal-status) :status))
      (case state
        (actionlib_msgs::recalling
         (setq state actionlib_msgs::pending))
        (actionlib_msgs::preempting
         (setq state actionlib_msgs::active)))
      state))
  (:get-goal-status-text () (send (send comm-state :latest-goal-status) :text))
  (:cancel-all-goals
   ()
   (let (cancel)
     (setq cancel (instance actionlib_msgs::GoalId :init
                            :id ""))
     (ros::publish (format nil "~A/cancel" name-space) cancel)))
  (:cancel-goal
   ()
   (let (cancel)
     (setq cancel (instance actionlib_msgs::GoalId :init
                            :id (send (send comm-state :action-goal) :goal_id :id)))
     (ros::publish (format nil "~A/cancel" name-space) cancel)
     (send comm-state :state 'ros::*comm-state-waiting-for-cancel-ack*)))
  ;;
  (:get-result-status
   ()
   (warning-message 1 "get-reuslt-status is obsolete method, use get-goal-status-text or get-goal-state")
   (send comm-state :latest-goal-status))
  )


(defclass ros::simple-action-server
  :super ros::object
  :slots (name-space action-spec status comm-state
                     action-goal-class action-result-class action-feedback-class
                     execute-cb goal goal-id seq-id))

(defmethod ros::simple-action-server
  (:execute-cb
   ()
   (let (msg status-list)
     (if (and goal execute-cb) (funcall execute-cb self goal))
     (setq msg (instance actionlib_msgs::GoalStatusArray :init))
     (when goal-id
       (send msg :status_list
             (list (instance actionlib_msgs::goalstatus :init
                             :goal_id goal-id
                             :status status
                             :text (goal-status-to-string status))))
       )
     (send msg :header :seq (send self :next-seq-id))
     (send msg :header :stamp (ros::time-now))
     (ros::publish (format nil "~A/status" name-space) msg)
     ))
  (:goal-callback
   (msg)
   (ros::ros-info ";; Recieved Goal ~A" (send msg :goal_id :id))
   (if (eq status actionlib_msgs::active) (send self :set-canceled))
   (setq status actionlib_msgs::active)
   (setq goal msg)
   (setq goal-id (send msg :goal_id)))
  (:cancel-callback
   (msg)
   (ros::ros-info ";; Recieved Cancel")
   (setq status actionlib_msgs::preempting)
   (send self :set-canceled)
   )
  (:set-succeeded
   (msg)
   (ros::ros-info ";; Send Result ~A" (send goal-id :id))
   (setq status actionlib_msgs::succeeded)
   (send msg :status :goal_id  goal-id)
   (send msg :status :status status)
   (send msg :header :stamp (ros::time-now))
   (ros::publish (format nil "~A/result" name-space) msg)
   (setq goal nil)
   (setq status actionlib_msgs::pending)
   )
  (:set-canceled
   (&optional (result (send self :result)))
   (ros::ros-info ";; Cancel Goal ~A" (send goal-id :id))
   (case status
     ((actionlib_msgs::pending actionlib_msgs::recalling)
      (setq status actionlib_msgs::recalled)
      )
     ((actionlib_msgs::active actionlib_msgs::preepmpting)
      (setq status actionlib_msgs::preempted)
      ))
    (send self :set-succeeded result))
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
  (:name-space () name-space)
  (:next-seq-id () (incf seq-id))
  )

(provide :actionlib "actionlib.l")
