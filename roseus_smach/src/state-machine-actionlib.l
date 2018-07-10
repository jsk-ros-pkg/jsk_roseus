(load "state-machine-ros.l")

;; binary state object
(defclass binary-state-object
  :super propertied-object
  :slots (enabled))
(defmethod binary-state-object
  (:init (&optional (initial-value nil))
   (setq enabled (not (null initial-value))))
  (:enable ()
   (setq enabled t))
  (:disable ()
   (setq enabled nil))
  (:enabled-p ()
   enabled))

;; execute state machine as action server

(defclass state-machine-action-server
  :super propertied-object
  :slots ((as :forward (:worker))
    sm userdata successes inspector))
(defmethod state-machine-action-server
  (:init
   (ns spec &optional (_sm nil) (_userdata '(nil)))
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
   (set-alist :cancel (instance binary-state-object :init) userdata)
   (set-alist :goal (send msg :goal) userdata)
   (set-alist :result (send (send as :result) :result) userdata)
   (set-alist :feedback (send (send as :feedback) :feedback) userdata)
   (send sm :reset-state))
  (:preempt-cb
   (msg)
   (send (cdr (assoc :cancel userdata)) :enable))
  (:execute-cb
   ()
   (cond
     ((null sm) nil)
     ((send as :is-active)
      (let ((result-msg (send as :result)))
        (cond
          ((send as :is-preempt-requested)
           ;; send incomplete data as result
           (ros::ros-info "Preempt requested")
           (send result-msg :result (cdr (assoc :result userdata)))
           (send as :set-preempted result-msg))
          ((send sm :goal-reached)
           (ros::ros-debug "Goal reached")
           (send result-msg :result (cdr (assoc :result userdata)))
           (if (member (send (send sm :active-state) :name) successes)
               (send as :set-succeeded result-msg)
               (send as :set-aborted result-msg)))
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
    (name client
     &key (timeout 10) (retry nil) (key #'identity) (return-success :succeeded) (return-fail :failed)
       (async nil) (return-async t))
  (send client :wait-for-server)
  (instance state :init name
   ;; main loop
   `(lambda (userdata)
      (let ((start (ros::time-now)) async-clients (last-warn (ros::time 0)))
        (when (or (null (cdr (assoc :cancel userdata)))
                  (not (derivedp (cdr (assoc :cancel userdata)) binary-state-object)))
          (ros::ros-warn "userdata :cancel is empty or not binary-state-object. Cancellation may not work."))
        (send ,client :send-goal
              (funcall (quote ,key) (cdr (assoc :goal userdata))))
        (if ,async
            (progn
              (setq async-clients (cdr (assoc :async userdata)))
              (set-alist :async (flatten (list async-clients ,client)) userdata)
              (ros::sleep)
              (send ,client :spin-once)
              ,return-async) ;; async end
            (while (ros::ok)
              (ros::sleep)
              (ros::spin-once) ;; spin to update user cancel state
              (send ,client :spin-once)
              ;;
              (cond
                ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*SUCCEEDED*)
                 (set-alist :result (send ,client :get-result) userdata)
                 (return ,return-success))
                ((eq (send ,client :get-state) actionlib_msgs::GoalStatus::*ABORTED*)
                 (if ,retry
                     (send ,client :send-goal
                           (funcall (quote ,key) (cdr (assoc :goal userdata))))
                     (progn
                       (set-alist :result (send ,client :get-result) userdata)
                       (return ,return-fail))))
                ((member (send ,client :get-state)
                         (list actionlib_msgs::GoalStatus::*PREEMPTED*
                               actionlib_msgs::GoalStatus::*RECALLED*
                               actionlib_msgs::GoalStatus::*REJECTED*))
                 (ros::ros-info "~A was canceled" (send ,client :name))
                 (set-alist :cancel t userdata)
                 (set-alist :result (send ,client :get-result) userdata)
                 (return ,return-fail))
                ((member (send ,client :get-state)
                         (list actionlib_msgs::GoalStatus::*PREEMPTING*
                               actionlib_msgs::GoalStatus::*RECALLING*))
                 (when (> (send (ros::time- (ros::time-now) last-warn) :to-sec) 1.0)
                   (ros::ros-info "Waiting for cancellation of ~A" (send ,client :name))
                   (setq last-warn (ros::time-now))))
                ((member (send ,client :get-state)
                         (list actionlib_msgs::GoalStatus::*ACTIVE*
                               actionlib_msgs::GoalStatus::*PENDING*))
                 ;; user cancel
                 (when (and (cdr (assoc :cancel userdata))
                            (derivedp (cdr (assoc :cancel userdata)) binary-state-object)
                            (send (cdr (assoc :cancel userdata)) :enabled-p))
                   (ros::ros-info "Sending cancel to ~A" (send ,client :name))
                   (send ,client :cancel-goal))
                 ;; time out
                 (when (and (numberp ,timeout)
                            (< ,timeout (send (ros::time- (ros::time-now) start) :to-sec)))
                     (ros::ros-info "Timed out. Sending cancel to ~A" (send ,client :name))
                     (send ,client :cancel-goal))))
              )))))
  )

;; simple usage
;; (instance state-machine-action-server :init "/action_name" awesome_pkg::GreatAction (your-state-machine) userdata)
;; (do-until-key (ros::spin-once) (send s :worker) (unix::usleep 10000))

;; reserved keyword in userdata
;; :goal,:feedback,:result variables in userdata are message defined in .action
;; :cancel variable in userdata is vector which (elt v 0) = t when preempt-requested

(defclass async-join-state
 :super state
 :slots (timeout))

(defmethod async-join-state
 (:init
  (name &key ((:timeout tm) 0) ((:remap-list rlst) nil))
  (send-super :init name nil)
  (setq timeout tm
        remap-list rlst))
 (:execute
  (userdata &key (step nil))
  (let (start-time ret clients)
    (send self :remap userdata :invert nil)
    (setq start-time (ros::time-now))
    (setq clients (cdr (assoc :async userdata)))
    (when (atom clients)
      (setq clients (list clients)))
    (warning-message 2 "waiting action client: ~A~%" clients)
    (send-all clients :spin-once)
    (if (member nil (send-all clients :wait-for-result :timeout timeout))
      (setq ret nil) (setq ret t))
    (warn "ret: ~A~%" ret)
    (set-alist :results (send-all clients :get-result) userdata)
    (warn "result: ~A~%" (send-all clients :get-result))
    (send self :remap userdata :invert t)
    (return-from :execute ret))))

(provide :state-machine-actionlib) ;; end of state-machine-actionlib.l
