(load "state-machine-ros.l")

;; execute state machine as action server

(defclass state-machine-action-server
  :super propertied-object
  :slots ((as :forward (:worker))
	  sm userdata successes inspector))
(defmethod state-machine-action-server
  (:init
   (ns spec &optional (_sm nil) (_userdata nil))
   (setq as (instance ros::simple-action-server :init ns spec
		      :execute-cb `(lambda(s g)(send ,self :execute-cb))
		      :accept-cb  `(lambda(s g)(send ,self :accept-cb g))
		      :preempt-cb `(lambda(s g)(send ,self :preempt-cb g)))
	 sm _sm userdata _userdata
	 inspector (instance state-machine-inspector :init _sm))
   (dolist (key '(:goal :feedback :result :cancel))
     (when (assoc key userdata)
       (warn "key ~A of userdata is already existed.~%")))
   (send self :success-state (send-all (send sm :goal-state) :name))
   self)
  ;; names of success state
  (:success-state
   (&optional states)
   (if states (setq successes states) successes))
  ;;
  (:accept-cb
   (msg)
   (setq userdata (set-alist :cancel (vector nil) userdata))
   (set-alist :goal (send msg :goal) userdata)
   (set-alist :result (send (send as :result) :result) userdata)
   (set-alist :feedback (send (send as :feedback) :feedback) userdata)
   (send sm :reset-state))
  (:preempt-cb
   (msg)
   (setf (elt (cdr (assoc :cancel userdata)) 0) t))
  (:execute-cb
   ()
   (cond
    ((null sm) nil)
    ((send as :is-active)
     (let ((result-msg (send as :result)))
       (cond
	((send sm :goal-reached)
	 (send result-msg :result (cdr (assoc :result userdata)))
	 (if (member (send (send sm :active-state) :name) successes)
	     (send as :set-succeeded result-msg)
	   (send as :set-aborted result-msg)))
	((send as :is-preempt-requested)
	 (send as :set-preempted result-msg))
	(t
	 (send inspector :publish-structure)
	 (send inspector :publish-status userdata)
	 (send sm :execute userdata :step -1)
	 (let ((feedback-msg (send as :feedback)))
	   (send feedback-msg :feedback (cdr (assoc :feedback userdata)))
	   (send as :publish-feedback feedback-msg)))
	)))
    (t nil)))
  )

;;
;; execute actionlib as a state machine
;; return 'state' instance
;;   this state needs :goal data, :cancel and :result are optional
;;   this state returns :succeeded or :failed
;;
(defun actionlib-client-state
  (name client &key (timeout 10) (retry nil) (key #'identity))
  (send client :wait-for-server)
  (instance state :init name
   ;; main loop
   `(lambda (userdata)
      (let ((start (ros::time-now)))
	(send ,client :send-goal
	      (funcall (quote ,key) (cdr (assoc :goal userdata))))
	(while (ros::ok)
	  (ros::sleep)
	  (ros::spin-once)
	  ;;
	  (cond
	   ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*SUCCEEDED*)
	     (return :succeeded))
	    ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*ABORTED*)
	     (if ,retry
		 (send ,client :send-goal
		       (funcall (quote ,key) (cdr (assoc :goal userdata))))
	       (return :failed)))
	    ((member (send ,client :get-state)
		     (list actionlib_msgs::GoalStatus::*PREEMPTED*
			   actionlib_msgs::GoalStatus::*RECALLED*
			   actionlib_msgs::GoalStatus::*REJECTED*))
	     (return :failed))
	    ((member (send ,client :get-state)
		     (list actionlib_msgs::GoalStatus::*ACTIVE*
			    actionlib_msgs::GoalStatus::*PENDING*))
	     ;; user cancel
	     (if (and (vectorp (cdr (assoc :cancel userdata)))
		      (elt (cdr (assoc :cancel userdata)) 0))
		 (send ,client :cancel-goal))
	     ;; time out
	     (if (and (numberp ,timeout)
		      (< ,timeout (send (ros::time- (ros::time-now) start) :to-sec)))
		 (send ,client :cancel-goal))))
	  ))))
  )

;; simple usage
;; (instance state-machine-action-server :init "/action_name" awesome_pkg::GreatAction (your-state-machine) userdata)
;; (do-until-key (ros::spin-once) (send s :worker) (unix::usleep 10000))

;; reserved keyword in userdata
;; :goal,:feedback,:result variables in userdata are message defined in .action
;; :cancel variable in userdata is vector which (elt v 0) = t when preempt-requested
