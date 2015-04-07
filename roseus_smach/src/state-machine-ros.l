;; state-machine-ros.l

(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :pickle "pickle.l")

(ros::roseus-add-msgs "smach_msgs")

;;
;; inspector publishes structure and status of state-machine
;; and it changes active-state of state-machine by init message
;;

(defclass state-machine-inspector
  :super propertied-object
  :slots (sm root-name srv-name init-tm state-counter structure-counter))
(defmethod state-machine-inspector
  (:init
   (sm-obj &key ((:root-name rn) "SM_ROOT") ((:srv-name sn) "/server_name"))
   (setq sm sm-obj root-name rn srv-name sn)
   (setq state-counter 0 structure-counter 0)
   (ros::advertise
    (concatenate string srv-name "/smach/container_status")
    smach_msgs::SmachContainerStatus 100)
   (ros::advertise
    (concatenate string srv-name "/smach/container_structure")
    smach_msgs::SmachContainerStructure 100 t) ;; latch type
   (ros::subscribe (concatenate string srv-name "/smach/container_init")
                   smach_msgs::SmachContainerInitialStatusCmd
                   #'send self :set-initial-state-cb)
   self)
  ;;
  (:state-machine (&rest args) (forward-message-to sm args))
  ;;
  (:publish-status
   (&optional (userdata nil) (machine sm) (path (format nil "/~A" root-name)))
   (let ((msg (instance smach_msgs::SmachContainerStatus :init))
         (initial (send-all (flatten (list (send machine :start-state))) :name))
         (active (send-all (flatten (list (send machine :active-state))) :name)))
     (if (send machine :goal-reached) (setq active "None"))
     (send msg :header :seq (incf state-counter))
     (send msg :header :stamp (ros::time-now))
     (send msg :path path)
     (send msg :initial_states (mapcar #'string initial))
     (send msg :active_states (mapcar #'string active))
     (send msg :local_data (pickle::dump userdata))
     (ros::publish (concatenate string srv-name "/smach/container_status") msg)
     ;; recursive call for publish active sub-machines
     (dolist (active-state (flatten (list (send machine :active-state))))
       (when (send active-state :submachine)
         (send self :publish-status userdata
               (send active-state :submachine)
               (format nil "~A/~A" path
                       (string (send active-state :name))))))
     ))
  (:publish-all-status
   (&optional (userdata nil) (machine sm) (path (format nil "/~A" root-name)))
   (let ((msg (instance smach_msgs::SmachContainerStatus :init))
         (initial (send-all (flatten (list (send machine :start-state))) :name))
         active)
     (cond
      ((send machine :goal-reached) (setq active "None"))
      ((null (send machine :active-state)) (setq active "None"))
      (t (setq active (send-all (flatten (list (send machine :active-state))) :name))))
     (send msg :header :seq (incf state-counter))
     (send msg :header :stamp (ros::time-now))
     (send msg :path path)
     (send msg :initial_states (mapcar #'string initial))
     (send msg :active_states (mapcar #'string active))
     (send msg :local_data (pickle::dump userdata))
     (ros::publish (concatenate string srv-name "/smach/container_status") msg)
     ;; recursive call for publish active sub-machines
     (dolist (nd (send machine :nodes))
       (let ((subsm (send nd :submachine)))
         (when subsm
           (send self :publish-all-status userdata subsm
                 (format nil "~A/~A" path
                         (string (send nd :name)))))))
     ))
  (:publish-structure
   (&optional (machine sm) (path (format nil "/~A" root-name)))
   (let ((msg (instance smach_msgs::SmachContainerStructure :init))
         transitions from-nodes to-nodes exec-nodes)
     (send msg :header :seq (incf structure-counter))
     (send msg :header :stamp (ros::time-now))
     (send msg :path path)
     (setq exec-nodes (remove-if #'(lambda(n)(send machine :goal-test n)) (send machine :nodes)))
     (send msg :children (mapcar #'string (send-all exec-nodes :name)))
     (send msg :container_outcomes
           (mapcar #'string (send-all (send machine :goal-state) :name)))
     (dolist (from (send machine :nodes))
       (dolist (tra (send from :arc-list))
         (push (string (send tra :name)) transitions)
         (push (string (send from :name)) from-nodes)
         (push (string (send (send tra :to) :name)) to-nodes)))
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
   (let ((path (send msg :path))
     ;; this smach takes only one state
     (name (car (send msg :initial_states))))
     ;; path should be start from root-name
     (when (not (string= (subseq path 1 (1+ (length root-name))) root-name))
       (return-from :set-initial-state-cb))
     (setq path (format nil "~A/~A"
                        (subseq path (1+ (length root-name))) name))
     ;; initialize can be called every 5 sec
     (when init-tm
       (when (< (send (ros::time- (ros::time-now) init-tm) :to-sec) 5)
         (return-from :set-initial-state-cb)))
     ;; parse the path string and get target node
     (let ((nd sm)
           (node-list (cdr (send (pathname path) :directory))))
       (dolist (node-name node-list)
         (when nd
           (setq nd (find node-name (send nd :nodes) :test #'string=
                                                     :key #'(lambda(x)(string (send x :name)))))
           (setq nd (send nd :submachine))))
       ;; set active-state
       (when nd
         (setq nd (find name (send nd :nodes) :test #'string=
                                              :key #'(lambda(x)(string (send x :name)))))
         (send sm :start-state nd)
         (send sm :reset-state)
         (ros::ros-info "Set initial/active state [~a]" nd)))
     ;; TODO apply userdata
     (setq init-tm (ros::time-now))
     ))
  ;;
  ;; utility for users
  ;;
  (:reset-state (&key (user-data) (on-state t))
   (cond
    ((and on-state
          (stringp on-state))
     (let ((st (send sm :node on-state)))
       (if st (send sm :active-state st)
         (progn
           (warn ";; node ~A not found" on-state)
           (send sm :reset-state)))))
    ((and on-state
          (derivedp on-state node))
     (send sm :active-state on-state))
    (on-state
     (send sm :reset-state)))
   (send self :publish-structure)
   (send self :publish-all-status user-data))
  (:exec-state-machine (&key (user-data) (reset-state))
   (send self :reset-state :user-data user-data :on-state reset-state)
   (while (not (send sm :goal-reached))
     (ros::spin-once)
     (send self :publish-status user-data)
     (send self :state-machine :execute user-data :step -1))
   (warn ";; goal reached at ~A~%" (send sm :active-state))
   (send self :publish-all-status user-data)
   (send sm :active-state))
  )

(provide :state-machine-ros)
