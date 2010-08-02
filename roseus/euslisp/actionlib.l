(if (not (find-package "ACTIONLIB_MSGS")) (roseus-add-msgs "actionlib_msgs"))

;;;; actionlib
(setq ros::*simple-goal-state-pending* 0
      ros::*simple-goal-state-active*  1
      ros::*simple-goal-state-done*    2)

(load "actionlib-commstate.l")

(defclass ros::simple-action-client
  :super ros::object
  :slots (name-space action-spec simple-state comm-state
                     action-goal-class action-result-class action-feedback-class
                     goal_id action-goal-msg))

(defmethod ros::simple-action-client
  (:goal-status-cb
   (msg)
   (let ((m (and goal_id
                 (find-if #'(lambda (m)
                              (string= (send m :goal_id :id)
                                       (send goal_id :id)))
                          (send msg :status_list)))))
     (when m
       (let ((com-state (send comm-state :update-state (send m :status)))
             (error-msg
              (format nil "Received comm state ~A when in simple state ~A"
                      comm-state simple-state)))
         ;;(warning-message 1 ";; com-state ~A~%" com-state)
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
   (let (com-state)
     (setq action-goal-msg msg)
     (setq com-state (send comm-state :state))
     (cond ((member com-state
                    (list 'ros::*comm-state-waiting-for-ack*
                          'ros::*comm-state-pending* 'ros::*comm-state-active*
                          'ros::*comm-state-waiting-for-result*
                          'ros::*comm-state-recalling*
                          'ros::*comm-state-preempting*))
            (send comm-state :state 'ros::*comm-state-done*))
           ((eq com-state 'ros::*comm-state-done*)
            (ros::ros-error "Get a result when we ware already in the DONE state"))
           (t
            (ros::ros-error (format nil "In a fanny state ~A" com-state))))
     action-goal-msg))
  (:action-feedback-cb (msg))
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
   ()
   (let ((goal-topic (format nil "~A/goal" name-space))
         (cancel-topic (format nil "~A/cancel" name-space)))
     (ros::rate 10)
     (while (and (= (ros::get-num-subscribers goal-topic) 0)
                 (= (ros::get-num-subscribers cancel-topic) 0))
       (ros::spin-once)
       (ros::sleep))))
  (:send-goal
   (goal)
   (let ((now (ros::time-now)))
     (setq goal_id (instance actionlib_msgs::GoalID :init
                             :id (format nil "~a" (send now :sec-nsec))))
     (send goal :goal_id goal_id)
     (setq simple-state ros::*simple-goal-state-pending*)
     (setq comm-state (instance actionlib-comm-state :init))
     (ros::publish (format nil "~A/goal" name-space) goal)))
  (:send-goal-and-wait
   (goal &key (timeout 0))
   (send self :send-goal goal)
   (send self :wait-for-result :timeout timeout))
  (:wait-for-result
   (&key (timeout 0)) ;; msec
   (ros::rate 10)
   (let ((start (send (ros::time-now) :sec-nsec)))
     (while (ros::ok)
       (if (= simple-state ros::*simple-goal-state-done*)
	   (return))
       (if (> timeout 0)
	   (let* ((tm (ros::time-now))
		  (iv (v- (send tm :sec-nsec) start)))
	     (if (> (+ (* 1000 (elt iv 0))
		       (/ (elt iv 1) 1000000))
		    timeout) (return))))
       (ros::spin-once)
       (ros::sleep)))
   simple-state)
  (:get-result ()  (if action-goal-msg (send action-goal-msg :result)))
  (:get-result-status ()  (if action-goal-msg (send action-goal-msg :status)))
  (:cancel-all-goals ()) ;; TODO  need sample program to test
  (:cancel-goal ())
  ;;
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
     (send msg :header :seq (incf seq-id))
     (send msg :header :stamp (ros::time-now))
     (ros::publish (format nil "~A/status" name-space) msg)
     ))
  (:goal-callback
   (msg)
   (ros::ros-info ";; Recieved Goal")
   (setq status actionlib_msgs::active)
   (setq goal msg)
   (setq goal-id (send msg :goal_id)))
  (:cancel-callback
   (msg)
   (ros::ros-info ";; Recieved Cancel")
   )
  (:set-succeeded
   (msg)
   (ros::ros-info ";; Send Result")
   (setq status actionlib_msgs::succeeded)
   (send msg :status :goal_id  goal-id)
   (send msg :status :status status)
   (send msg :header :stamp (ros::time-now))
   (ros::publish (format nil "~A/result" name-space) msg)
   (setq goal nil)
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
  (:name-space () name-space)
  (:next-seq-id () (incf seq-id))
  )

