(load "state-machine.l")
(require :pickle "pickle.l")

(ros::roseus-add-msgs "smach_msgs")

(defclass state-machine-inspector
  :super propertied-object
  :slots (sm root-name srv-name init-tm))
(defmethod state-machine-inspector
  (:init
   (sm-obj &key ((:root-name rn) "SM_ROOT") ((:srv-name sn) "/server_name"))
   (setq sm sm-obj root-name rn srv-name sn)
   (ros::advertise
    (concatenate string srv-name "/smach/container_status")
    smach_msgs::SmachContainerStatus 10)
   (ros::advertise
    (concatenate string srv-name "/smach/container_structure")
    smach_msgs::SmachContainerStructure 10)
   (ros::subscribe (concatenate string srv-name "/smach/container_init")
		   smach_msgs::SmachContainerInitialStatusCmd
		   #'send self :set-initial-state-cb 1)
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
     (ros::publish (concatenate string srv-name "/smach/container_status") msg)
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
     (ros::publish (concatenate string srv-name "/smach/container_structure") msg))
     ;; recursive call for publish sub-machines
     (dolist (sub (send machine :sub-sm-node))
       (send self :publish-structure (send sub :submachine)
	     (format nil "~A/~A" path (string (send sub :name)))))
     )
  (:set-initial-state-cb
   (msg)
   (let (nd (path (send msg :path))
	 (name (car (send msg :initial_states)))) ;; this smach takes only one state
     ;; initialize can be called every 5 sec
     (when init-tm
       (when (< (send (ros::time- (ros::time-now) init-tm) :to-sec) 5)
	 (return-from :set-initial-state-cb)))
     ;; TODO enable to parse the path string
     (setq nd (find name (send sm :nodes)
		    :key #'(lambda(x)(send x :name))
		    :test #'string=))
     (when nd
       (send sm :start-state nd)
       (send sm :reset-state))
     (ros::ros-info "Set initial state")
     (setq init-tm (ros::time-now))
     ))
  )
