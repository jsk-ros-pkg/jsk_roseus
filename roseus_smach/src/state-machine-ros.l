(load "state-machine.l")
(require :pickle "pickle.l")

(ros::roseus-add-msgs "smach_msgs")

(defclass state-machine-inspector
  :super propertied-object
  :slots (sm root-name))
(defmethod state-machine-inspector
  (:init
   (sm-obj &key ((:root-name rn) "SM_ROOT"))
   (setq sm sm-obj root-name rn)
   (ros::advertise "/server_name/smach/container_status"
		   smach_msgs::SmachContainerStatus 10)
   (ros::advertise "/server_name/smach/container_structure"
		   smach_msgs::SmachContainerStructure 10)
   self)
  ;;
  (:publish-status
   (&optional (userdata nil) (machine sm) (path (format nil "/~A" root-name)))
   (let ((msg (instance smach_msgs::SmachContainerStatus :init))
	 (initial (send (send machine :start-state) :name))
	 (active (send (send machine :active-state) :name)))
     (if (send machine :goal-reached) (setq active "None"))
     (send msg :header :stamp (ros::time-now))
     (send msg :path path)
     (send msg :initial_states (list (string initial)))
     (send msg :active_states (list (string active)))
     (send msg :local_data (pickle::dump userdata))
     (ros::publish "/server_name/smach/container_status" msg)
     ;; recursive call for publish active sub-machines
     (when (send (send machine :active-state) :submachine)
       (send self :publish-status userdata
	     (send (send machine :active-state) :submachine)
	     (format nil "~A/~A" path
		     (string (send (send machine :active-state) :name)))))
     ))
  (:publish-structure
   (&optional (machine sm) (path (format nil "/~A" root-name)))
   (let ((msg (instance smach_msgs::SmachContainerStructure :init))
	 transitions from-nodes to-nodes exec-nodes)
     (send msg :header :stamp (ros::time-now))
     (send msg :path path)
     (setq exec-nodes (remove-if #'(lambda(n)(send machine :goal-test n)) (send machine :nodes)))
     (send msg :children
	   (mapcar #'string (send-all exec-nodes :name)))
     (send msg :container_outcomes
	   (mapcar #'string (send-all (send machine :goal-state) :name)))
     (dolist (from (send machine :nodes))
       (dolist (tra (send from :transitions))
	 (push (string (car tra)) transitions)
	 (push (string (send from :name)) from-nodes)
	 (push (string (send (cdr tra) :name)) to-nodes)))
     (send msg :internal_outcomes transitions)
     (send msg :outcomes_from from-nodes)
     (send msg :outcomes_to to-nodes)
     (ros::publish "/server_name/smach/container_structure" msg)
     ;; recursive call for publish sub-machines
     (dolist (sub (send machine :sub-sm-node))
       (send self :publish-structure (send sub :submachine)
	     (format nil "~A/~A" path (string (send sub :name)))))
     ))
  )

;; execute state machine as action server
;; (Maniptask action server Compatible, atm_task_controller)
(defclass state-machine-action-server
  :super propertied-object
  :slots ((ac :forward (:set-succeeded :set-aborted :set-preempted
			:publish-feedback :goal :feedback :result
			:worker :is-active :is-preempt-requested))
	  sm userdata successes inspector))
(defmethod state-machine-action-server
  (:init
   (ns spec &optional (_sm nil) (_userdata nil))
   (setq ac (instance ros::simple-action-server :init ns spec
		      :execute-cb `(lambda(s g)(send ,self :execute-cb))
		      :accept-cb  `(lambda(s g)(send ,self :accept-cb g))
		      :preempt-cb `(lambda(s g)(send ,self :preempt-cb g)))
	 sm _sm userdata _userdata
	 inspector (instance state-machine-inspector :init _sm))
   (setq userdata (set-alist :cancel nil userdata)) ;; userdata may be nil
   (send self :success-state (send sm :goal-state)) ;; all terminal-state(temp)
   self)
  (:success-state
   (&optional states)
   (if states (setq successes states) successes))
  (:accept-cb
   (msg)
   (set-alist :goal (send msg :goal) userdata)
   (set-alist :result (send (send self :result) :result) userdata)
   (set-alist :feedback (send (send self :feedback) :feedback) userdata)
   (send sm :reset-state))
  (:preempt-cb
   (msg)
   (setf (cdr (assoc :cancel userdata)) (vector nil)))
  (:execute-cb
   ()
   (cond
    ((null sm) nil)
    ((send self :is-active)
     (let ((result-msg (send self :result)))
       (cond
	((send sm :goal-reached)
	 (send result-msg :result (cdr (assoc :result userdata)))
	 (if (member (send (send sm :active-state) :name) successes)
	     (send self :set-succeeded result-msg)
	   (send self :set-aborted result-msg)))
	((send self :is-preempt-requested)
	 (send self :set-preempted result-msg))
	(t
	 (send inspector :publish-structure)
	 (send inspector :publish-status userdata)
	 (send sm :execute userdata :step -1)
	 (let ((feedback-msg (send self :feedback)))
	   (send feedback-msg :feedback (cdr (assoc :feedback userdata)))
	   (send self :publish-feedback feedback-msg)))
	)))
    (t nil)))
  )

;; simple usage
;; (instance state-machine-action-server :init "/action_name")
;; (do-until-key (send s :worker))
