(unless (find-package "ROSEUS_BT")
  (make-package "ROSEUS_BT" :nicknames (list "ROSEUS-BT")))

(in-package "ROSEUS_BT")

(export '(action-node condition-node cancel-action spin-once spin))
;; import relevant ros::simple-action-server symbols
(import '(ros::name-space ros::status ros::execute-cb
          ros::goal ros::goal-id))

(defvar *action-list*)
(defvar *condition-list*)


(defcondition cancel-action :slots (server goal))

(defun signals-cancel (server goal)
  (signals cancel-action :server server :goal goal))

(defun check-goal (server new-goal old-goal)
  (equal (send new-goal :goal) (send old-goal :goal)))

;; classes
(defclass action-node :super ros::simple-action-server :slots (monitor-groupname))
(defmethod action-node
  (:init (ns spec &key execute-cb (preempt-cb 'signals-cancel) accept-cb groupname
             ((:monitor-groupname monitor-gn)))

      (send-super :init ns spec
              :execute-cb execute-cb
              :preempt-cb preempt-cb
              :accept-cb accept-cb
              :groupname groupname)
      (when monitor-gn
        (setq monitor-groupname monitor-gn)
        (ros::create-nodehandle monitor-groupname)
        ;; resubscribe /cancel on separate handler
        (ros::subscribe (format nil "~A/cancel" ns)
                        actionlib_msgs::GoalID #'send self :cancel-callback 50
                        :groupname monitor-groupname))

      ;; Publishing the status topic is required to make a
      ;; connection with the action client, but doing so on
      ;; high frequencies is slow. Throttle it to 5Hz.
      (ros::create-timer 0.2 #'(lambda (event) (send self :publish-status))
                         :groupname groupname)

      (push self *action-list*)
      (if *condition-list*
          (ros::ros-warn "Actions and Conditions detected in the same node! Start two separate nodes when reactivity is required."))
      self)
  (:publish-status ()
    (let ((msg (instance actionlib_msgs::GoalStatusArray :init)))
      (when goal-id
        (send msg :status_list
              (list (instance actionlib_msgs::goalstatus :init
                              :goal_id goal-id
                              :status status
                              :text (ros::goal-status-to-string status)))))
      (send msg :header :seq (send self :next-seq-id))
      (send msg :header :stamp (ros::time-now))
      (ros::publish (format nil "~A/status" name-space) msg)))
  (:execute-cb ()
    (when (and goal execute-cb)
      (let ((res (funcall execute-cb self goal)))
        (when (send self :is-active)
          (send self :set-succeeded
                (send self :result :success (not (not res))))))))
  (:set-output (name val)
    (let ((msg (send self :feedback (intern (string-upcase name) *keyword-package*) val)))
      (send msg :feedback :update_field_name name)
      (send self :publish-feedback msg)))
  (:ok ()
     (send self :spin-monitor)
     (not (send self :is-preempt-requested)))
  (:spin-monitor ()
    (if monitor-groupname
        (ros::spin-once monitor-groupname)
        (send self :spin-once))))


(defclass condition-node :super propertied-object :slots (execute-cb groupname))
(defmethod condition-node
  (:init (ns spec &key ((:execute-cb exec-cb)) ((:groupname gn)))
    (setq execute-cb exec-cb)
    (setq groupname gn)
    (if groupname
        (ros::advertise-service ns spec #'send self :request-cb :groupname groupname)
        (ros::advertise-service ns spec #'send self :request-cb))
    (push self *condition-list*)
    (if *action-list*
        (ros::ros-warn "Actions and Conditions detected in the same node! Start two separate nodes when reacitivity is required."))
    self)
  (:request-cb (msg)
    (let ((response (send msg :response)))
      (send response :success (not (not (funcall execute-cb self msg))))
      response))
  (:spin-once ()
    (if groupname
        (ros::spin-once groupname)
        (ros::spin-once))))


;; functions
(defun spin-once ()
  (dolist (cc *condition-list*)
    (send cc :spin-once))
  (dolist (ac *action-list*)
    (send ac :spin-once)
    (send ac :execute-cb)))

(defun spin ()
  (while (ros::ok)
    (spin-once)
    (ros::sleep)))
