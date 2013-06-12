(in-package "ROS")

(if (not (find-package "ACTIONLIB_MSGS")) (ros::roseus-add-msgs "actionlib_msgs"))

;;;; actionlib
(setq ros::*simple-goal-state-pending* 0
      ros::*simple-goal-state-active*  1
      ros::*simple-goal-state-done*    2)
(defun simple-goal-states-to-string (i)
  (elt '(ros::*simple-goal-state-pending* ros::*simple-goal-state-active* ros::*simple-goal-state-done*) i))

(load "actionlib-commstate.l")

(defclass ros::simple-action-client
  :super ros::object
  :slots (name-space action-spec simple-state comm-state status-stamp
                     action-goal-class action-result-class action-feedback-class
                     action-done-cb action-active-cb action-feedback-cb
                     goal_id groupname seq-count))

(defmethod ros::simple-action-client
  (:goal-status-cb
   (msg)
   ;;(ros::ros-debug "[~A] status-cb (seq:~A ~A)" name-space (send msg :header :seq) (mapcar #'(lambda (s) (format nil "id:~A, status:~A" (send s :goal_id :id) (goal-status-to-string (send s :status)))) (send msg :status_list)))
   (let ((m (send comm-state :find-status-by-goal-id msg)))
     (when m
       (let ((com-state (send comm-state :update-status msg))
             (error-msg
              (format nil "Received comm state ~A when in simple state ~A"
                      comm-state simple-state)))
         (ros::ros-debug "[~A] status-cb  (seq:~A ~A)" name-space (send msg :header :seq) (mapcar #'(lambda (s) (format nil "id:~A, status:~A" (send s :goal_id :id) (goal-status-to-string (send s :status)))) (send msg :status_list)))
	 (setq status-stamp (ros::time-now))
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
   (let (dummy-msg)
     (ros::ros-debug "[~A] reuslt-cb ~A ~A" name-space (send msg :status :goal_id :id) (goal-status-to-string (send msg :status :status)))
     (send comm-state :update-result msg)
     (ros::ros-debug "[~A] reuslt-cb ~A -> state ~A" name-space (send msg :status :goal_id :id) (send comm-state :state))
     ;; transition to goal state
     (setq dummy-msg (instance actionlib_msgs::GoalStatusArray :init))
     (send dummy-msg :status_list (list (send msg :status)))
     (send dummy-msg :header (send msg :header))
     (send self :goal-status-cb dummy-msg)
     ))
  (:action-feedback-cb
   (msg)
   (ros::ros-debug "[~A] feedback-cb ~A" name-space msg)
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
   (ns spec &key ((:groupname gp)))
   (send self :name ns)
   (setq name-space ns)
   (setq groupname gp)
   (setq seq-count 0)
   (if groupname (ros::create-nodehandle groupname))
   (setq action-spec (instance spec :init))
   (setq comm-state (instance actionlib-comm-state :init :name name-space))
   (setq action-goal-class     (class (send action-spec :action_goal))
         action-result-class   (class (send action-spec :action_result))
         action-feedback-class (class (send action-spec :action_feedback)))
   (ros::advertise (format nil "~A/goal" name-space) action-goal-class 0)
   (ros::advertise (format nil "~A/cancel" name-space) actionlib_msgs::GoalID 0)
   (cond
    (groupname
     (ros::subscribe (format nil "~A/status" name-space)
                     actionlib_msgs::GoalStatusArray #'send self :goal-status-cb 0 :groupname groupname)
     (ros::subscribe (format nil "~A/result" name-space)
                     action-result-class #'send self :action-result-cb 0 :groupname groupname)
     (ros::subscribe (format nil "~A/feedback" name-space)
                     action-feedback-class #'send self :action-feedback-cb 0 :groupname groupname))
    (t
     (ros::subscribe (format nil "~A/status" name-space)
                     actionlib_msgs::GoalStatusArray #'send self :goal-status-cb 0)
     (ros::subscribe (format nil "~A/result" name-space)
                     action-result-class #'send self :action-result-cb 0)
     (ros::subscribe (format nil "~A/feedback" name-space)
                     action-feedback-class #'send self :action-feedback-cb 0)))
   (setq simple-state ros::*simple-goal-state-done*)
   self)
  (:wait-for-server
   (&optional (timeout nil))
   (let ((goal-topic (format nil "~A/goal" name-space))
         (cancel-topic (format nil "~A/cancel" name-space))
         (feedback-topic (format nil "~A/feedback" name-space))
         (result-topic (format nil "~A/result" name-space))
	 (count 0))
     (ros::ros-debug "[~A] wait-for-server" name-space)
     (ros::rate 10)
     (while (or (= (ros::get-num-subscribers goal-topic) 0)
                (= (ros::get-num-subscribers cancel-topic) 0)
                (= (ros::get-num-publishers feedback-topic) 0)
                (= (ros::get-num-publishers result-topic) 0))
       (ros::ros-debug "[~A] num subscribers ~A ~d, ~A ~d" name-space
		       goal-topic (ros::get-num-subscribers goal-topic)
		       cancel-topic (ros::get-num-subscribers cancel-topic))
       (send self :spin-once)
       (ros::sleep)
       (when (and timeout (>= count (* timeout 10)))
         (ros::ros-warn "[~A] action server is not found" name-space)
         (ros::ros-warn "     goal=~d, cancel=~d, feedback=~d, result=~d"
                        (ros::get-num-subscribers goal-topic)
                        (ros::get-num-subscribers cancel-topic)
                        (ros::get-num-publishers feedback-topic)
                        (ros::get-num-publishers result-topic))
         (return-from :wait-for-server nil))
       (incf count))
     t))
  (:send-goal
   (goal &key done-cb active-cb feedback-cb)
   (let ((now (ros::time-now)))
     (setq goal_id (instance actionlib_msgs::GoalID :init
                             :id (format nil "~A_~A_~A_~A" (ros::get-name) (unix::getpid) name-space seq-count)))
     (incf seq-count)
     (send goal :goal_id goal_id)
     (setq simple-state ros::*simple-goal-state-pending*)
     (setq status-stamp nil)
     (setq comm-state (instance actionlib-comm-state :init :name name-space :action-goal goal :action-result (instance action-result-class :init)))
     (if done-cb (setq action-done-cb done-cb))
     (if active-cb (setq action-active-cb active-cb))
     (if feedback-cb (setq action-feedback-cb feedback-cb))
     (ros::ros-debug "[~A] send-goal ~A" name-space (send goal_id :id))
     (ros::publish (format nil "~A/goal" name-space) goal)))
  (:send-goal-and-wait
   (goal &key (timeout 0))
   (send self :send-goal goal)
   (unless (send self :wait-for-result :timeout timeout)
     (send self :cancel-goal)
     (send self :wait-for-result :timeout timeout)
     ))
  (:wait-for-result
   (&key (timeout 0) ;; sec
	 (return-if-server-down) (maximum-status-interval 5))
   (let ((start (ros::time-now))
	 (result-topic (format nil "~A/result" name-space)))
     (ros::ros-debug "[~A] :wait-for-result ~A, timeout=~A" name-space (if goal_id (send goal_id :id)) timeout)
     (ros::rate 10)
     (while (ros::ok)
       (ros::ros-debug "[~A] :wait-for-result ~A ~A" name-space (simple-goal-states-to-string simple-state) (send comm-state :state))
       (if (= simple-state ros::*simple-goal-state-done*)
	   (return))
       (if (> timeout 0)
	   (let* ((tm (ros::time- (ros::time-now) start)))
	     (if (> (send tm :to-sec) timeout) (return-from :wait-for-result nil))))
       (when return-if-server-down
	 (when (= (ros::get-num-publishers result-topic) 0)
	   (ros::ros-error "[~A] Unexpected returns from :wait-for-result : no publishers found for ~A" name-space result-topic)
	   (return-from :wait-for-result nil))
	 (when (and status-stamp maximum-status-interval (> (send (ros::time- (ros::time-now) status-stamp) :to-sec) maximum-status-interval))
	   (ros::ros-error "[~A] Unexpected returns from :wait-for-result : status did not received for 5 seconds" name-space)
	   (return-from :wait-for-result nil)))
       (send self :spin-once)
       (ros::sleep))
     (ros::ros-debug "[~A] :wait-for-result finished ~A" name-space (goal-status-to-string (send self :get-state)))
     (eq (send self :get-state) actionlib_msgs::GoalStatus::*succeeded*)))
  (:get-result () (send (send comm-state :latest-result) :result))
  (:get-state ()
    (let (state)
      (setq state (send (send comm-state :latest-goal-status) :status))
      (case state
        (actionlib_msgs::GoalStatus::*recalling*
         (setq state actionlib_msgs::GoalStatus::*pending*))
        (actionlib_msgs::GoalStatus::*preempting*
         (setq state actionlib_msgs::GoalStatus::*active*)))
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
  (:spin-once ()
   (cond
    (groupname (ros::spin-once groupname))
    (t (ros::spin-once))))
  ;;
  )


(defclass ros::simple-action-server
  :super ros::object
  :slots (name-space action-spec status
                     action-goal-class action-result-class action-feedback-class
                     execute-cb accept-cb preempt-cb
                     goal goal-id pending-goal
                     seq-id feed-seq-id
                     groupname))

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
  (:goal-callback ;; goal is always accepted, not rejected
   (msg)
   (ros::ros-info ";; Received Goal ~A" (send msg :goal_id :id))
   (cond (pending-goal ;; recall this
	  (let ((recall-msg (send self :result)))
	    (send recall-msg :status :goal_id (send pending-goal :goal_id))
	    (send recall-msg :status :status actionlib_msgs::GoalStatus::*recalled*)
	    (send recall-msg :header :stamp (ros::time-now))
	    (ros::publish (format nil "~A/result" name-space) recall-msg))
	  (setq pending-goal msg))
	 ((send self :is-active)
	  (setq status actionlib_msgs::GoalStatus::*preempting*)
	  (if preempt-cb (funcall preempt-cb self goal))
	  (setq pending-goal msg))
	 (t
	  (setq status actionlib_msgs::GoalStatus::*active*)
	  (setq goal msg goal-id (send msg :goal_id))
	  (if accept-cb (funcall accept-cb self goal)))
	 ))
  (:cancel-callback
   (msg)
   (ros::ros-info ";; Recieved Cancel")
   (setq check-func
	 #'(lambda(msg goal)
	     (let ((stamp (send goal :header :stamp)))
	       (and (member (send msg :id) (list "" (send goal :goal_id :id)) :test #'string=)
		    (or (= 0 (send (send msg :stamp) :to-sec))
			(< 0 (send (ros::time- (send msg :stamp) stamp) :to-sec)))))))
   ;;
   (when (and pending-goal (funcall check-func msg pending-goal))
     (setq pending-goal nil))
   (when (and (equal status actionlib_msgs::GoalStatus::*active*)
	      (funcall check-func msg goal))
     (setq status actionlib_msgs::GoalStatus::*preempting*)
     (if preempt-cb (funcall preempt-cb self goal))))
  ;;
  (:publish-result
   (msg &optional (text ""))
   (ros::ros-info ";; Send Result ~A" (send goal-id :id))
   (send msg :status :goal_id goal-id)
   (send msg :status :status status)
   (send msg :status :text text)
   (send msg :header :stamp (ros::time-now))
   (ros::publish (format nil "~A/result" name-space) msg)
   (setq status actionlib_msgs::GoalStatus::*pending*)
   (let ((next-goal pending-goal))
     (setq goal nil pending-goal nil)
     (if next-goal (send self :goal-callback next-goal))))
  (:publish-feedback
   (msg)
   (send msg :status :goal_id goal-id)
   (send msg :status :status status)
   (send msg :header :stamp (ros::time-now))
   (send msg :header :seq (send self :next-feed-seq-id))
   (ros::publish (format nil "~A/feedback" name-space) msg))
   ;;
  (:set-succeeded
   (msg &optional (text ""))
   (ros::ros-info ";; Action Succeeded ~A" (send goal-id :id))
   (setq status actionlib_msgs::GoalStatus::*succeeded*)
   (send self :publish-result msg text))
  (:set-aborted
   (msg &optional (text ""))
   (ros::ros-info ";; Action Aborted ~A" (send goal-id :id))
   (setq status actionlib_msgs::GoalStatus::*aborted*)
   (send self :publish-result msg text))
  (:set-preempted
   (&optional (msg (send self :result)) (text ""))
   (ros::ros-info ";; Cancel Goal ~A" (send goal-id :id))
   (case status
     ((actionlib_msgs::GoalStatus::*pending* actionlib_msgs::GoalStatus::*recalling*)
      (setq status actionlib_msgs::GoalStatus::*recalled*)
      )
     ((actionlib_msgs::GoalStatus::*active* actionlib_msgs::GoalStatus::*preepmpting*)
      (setq status actionlib_msgs::GoalStatus::*preempted*)
      ))
   (send self :publish-result msg text))
  ;;
  (:goal () goal)
  (:result
   (&rest args)
   (let (msg)
     (setq msg (instance action-result-class :init))
     (send* msg :result args)
     msg))
  (:feedback
   (&rest args)
   (let (msg)
     (setq msg (instance action-feedback-class :init))
     (send* msg :feedback args)
     msg))
  ;;
  (:init
   (ns spec &key ((:execute-cb exec-f)) ((:preempt-cb preempt-f))
       ((:groupname gp)) ((:accept-cb accept-f)))
   (send self :name ns)
   (setq name-space ns)
   (setq action-spec (instance spec :init))
   (setq execute-cb exec-f preempt-cb preempt-f accept-cb accept-f)
   ;;
   (setq seq-id 0 feed-seq-id 0)
   (setq status actionlib_msgs::GoalStatus::*pending*)
   ;;
   (setq groupname gp)
   (if groupname (ros::create-nodehandle groupname))
   ;;
   (setq action-goal-class     (class (send action-spec :action_goal))
         action-result-class   (class (send action-spec :action_result))
         action-feedback-class (class (send action-spec :action_feedback)))
   (ros::advertise (format nil "~A/status" name-space) actionlib_msgs::GoalStatusArray)
   (ros::advertise (format nil "~A/result" name-space) action-result-class)
   (ros::advertise (format nil "~A/feedback" name-space) action-feedback-class)
   (cond
    (groupname
     (ros::subscribe (format nil "~A/goal" name-space)
                     action-goal-class #'send self :goal-callback 0 :groupname groupname)
     (ros::subscribe (format nil "~A/cancel" name-space)
                     actionlib_msgs::GoalID #'send self :cancel-callback 0 :groupname groupname))
    (t
     (ros::subscribe (format nil "~A/goal" name-space)
                     action-goal-class #'send self :goal-callback 0)
     (ros::subscribe (format nil "~A/cancel" name-space)
                     actionlib_msgs::GoalID #'send self :cancel-callback 0)))
   ;; need getparam for status_frequence, status_list_timeout
   )
  ;;
  (:worker () (send self :execute-cb))
  (:is-preempt-requested () (eq status actionlib_msgs::GoalStatus::*preempting*))
  (:is-active () (memq status (list actionlib_msgs::GoalStatus::*preempting* actionlib_msgs::GoalStatus::*active*)))
  (:name-space () name-space)
  (:next-seq-id () (incf seq-id))
  (:next-feed-seq-id () (incf feed-seq-id))
  (:spin-once ()
   (cond
    (groupname (ros::spin-once groupname))
    (t (ros::spin-once))))
  )

(provide :actionlib "actionlib.l")
