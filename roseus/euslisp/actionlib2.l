;;
;; actionlib2.l - ROS 2 Action Client/Server for EusLisp
;;
;; ROS 1 compatible API using ROS 2 rcl_action bindings.
;;

(in-package "ROS")

;; Load action_msgs and unique_identifier_msgs if not loaded yet
(if (not (find-package "ACTION_MSGS"))
  (ros::load-ros-package "action_msgs"))
(if (not (find-package "UNIQUE_IDENTIFIER_MSGS"))
  (ros::load-ros-package "unique_identifier_msgs"))

;; Goal state constants (matching action_msgs/msg/GoalStatus)
(defconstant ros::*action-goal-state-unknown*   0)
(defconstant ros::*action-goal-state-accepted*  1)
(defconstant ros::*action-goal-state-executing* 2)
(defconstant ros::*action-goal-state-canceling* 3)
(defconstant ros::*action-goal-state-succeeded* 4)
(defconstant ros::*action-goal-state-canceled*  5)
(defconstant ros::*action-goal-state-aborted*   6)

;; Goal events for server-side state transitions
(defconstant ros::*goal-event-execute*     0)
(defconstant ros::*goal-event-cancel-goal* 1)
(defconstant ros::*goal-event-succeed*     2)
(defconstant ros::*goal-event-abort*       3)
(defconstant ros::*goal-event-canceled*    4)

(defun ros::generate-uuid ()
  "Generate a random UUID v4 as a 16-byte string."
  (let ((uuid (make-array 16 :element-type :char)))
    (dotimes (i 16) (setf (elt uuid i) (random 256)))
    ;; Version 4
    (setf (elt uuid 6) (logior #x40 (logand (elt uuid 6) #x0f)))
    ;; Variant 10xx
    (setf (elt uuid 8) (logior #x80 (logand (elt uuid 8) #x3f)))
    uuid))

(defun ros::uuid-to-unique-identifier-msg (uuid-bytes)
  "Create a unique_identifier_msgs/UUID instance from 16-byte array."
  (let ((msg (instance unique_identifier_msgs::UUID :init)))
    (send msg :uuid uuid-bytes)
    msg))

(defclass ros::simple-action-client
    ;; Simple Action Client for ROS 1 compatibility
  :super ros::object
  :slots (name-space action-spec action-type-str
          goal-class result-class feedback-class
          send-goal-service get-result-service feedback-message-class
          ;; current goal state
          current-uuid current-goal-status
          last-result last-feedback
          action-done-cb action-feedback-cb
          groupname))

(defmethod ros::simple-action-client
  (:init
   (ns spec &key ((:groupname gp)))
   "Initialize simple action client.
    ns: action namespace (e.g. \"fibonacci\")
    spec: action class (e.g. roseus::Fibonacci)"
   (setq name-space ns)
   (setq action-spec spec)
   (setq groupname gp)
   ;; Resolve type string from action spec
   (setq action-type-str (get spec :datatype-))
   ;; Get sub-type classes
   (setq goal-class (get spec :goal))
   (setq result-class (get spec :result))
   (setq feedback-class (get spec :feedback))
   (setq send-goal-service (get spec :send-goal-service))
   (setq get-result-service (get spec :get-result-service))
   (setq feedback-message-class (get spec :feedback-message))
   ;; Initialize state
   (setq current-uuid nil)
   (setq current-goal-status ros::*action-goal-state-unknown*)
   (setq last-result nil)
   (setq last-feedback nil)
   ;; Initialize rcl_action client
   (ros::action-client-init name-space action-type-str)
   self)
  ;;
  (:wait-for-server
   (&optional (timeout nil))
   "Wait for the action server to become available."
   (ros::action-wait-for-server name-space timeout))
  ;;
  (:send-goal
   (goal &key done-cb feedback-cb)
   "Send a goal to the action server.
    goal: Goal instance or SendGoalRequest (like ROS 1 ActionGoal).
    If goal has :goal accessor (SendGoalRequest), the inner goal is extracted.
    done-cb: callback when goal completes (funcall done-cb state result)
    feedback-cb: callback for feedback (funcall feedback-cb feedback)"
   (setq action-done-cb done-cb)
   (setq action-feedback-cb feedback-cb)
   (setq current-goal-status ros::*action-goal-state-unknown*)
   (setq last-result nil)
   (setq last-feedback nil)
   ;; Accept both Goal and SendGoalRequest (ROS 1 ActionGoal compat)
   (let ((actual-goal (if (derivedp goal goal-class) goal
                        (send goal :goal))))
     ;; Generate UUID
     (setq current-uuid (ros::generate-uuid))
     ;; Build SendGoal_Request
     (let ((req (instance (get send-goal-service :request) :init)))
       (send req :goal_id (ros::uuid-to-unique-identifier-msg current-uuid))
       (send req :goal actual-goal)
       ;; Send goal request
       (let ((seq (ros::action-send-goal name-space req)))
         (when seq
           (setq current-goal-status ros::*action-goal-state-accepted*))
         seq))))
  ;;
  (:spin-once
   ()
   "Poll for feedback, status, and responses."
   ;; Process pending callbacks via spin
   (if groupname
       (ros::spin-once groupname)
     (ros::spin-once))
   ;; Try to take goal response
   (let ((resp (instance (get send-goal-service :response) :init)))
     (when (ros::action-take-goal-response name-space resp)
       (unless (send resp :accepted)
         (setq current-goal-status ros::*action-goal-state-aborted*)
         (ros::ros-warn "[~A] goal rejected" name-space))))
   ;; Try to take feedback
   (let ((fb-msg (instance feedback-message-class :init)))
     (when (ros::action-take-feedback name-space fb-msg)
       (setq last-feedback (send fb-msg :feedback))
       (when action-feedback-cb
         (funcall action-feedback-cb last-feedback))))
   ;; Try to take status
   (let ((status-msg (instance action_msgs::GoalStatusArray :init)))
     (when (ros::action-take-status name-space status-msg)
       ;; Find our goal's status
       (dolist (s (send status-msg :status_list))
         (let ((uuid-bytes (send (send s :goal_info) :goal_id :uuid)))
           (when (and current-uuid (equal uuid-bytes current-uuid))
             (setq current-goal-status (send s :status)))))))
   ;; Try to take cancel response (must be consumed after cancel-goal)
   (let ((resp (instance action_msgs::CancelGoalResponse :init)))
     (ros::action-take-cancel-response name-space resp))
   ;; Try to take result response
   (let ((resp (instance (get get-result-service :response) :init)))
     (when (ros::action-take-result-response name-space resp)
       (setq last-result (send resp :result))
       (setq current-goal-status (send resp :status))
       (when action-done-cb
         (funcall action-done-cb current-goal-status last-result)))))
  ;;
  (:wait-for-result
   (&key (timeout nil))
   "Wait for the action result. Returns t if result received, nil if timeout."
   ;; If result already cached, return immediately
   (when last-result (return-from :wait-for-result t))
   ;; If no goal has been sent, return nil
   (when (null current-uuid) (return-from :wait-for-result nil))
   ;; Send result request
   (let ((req (instance (get get-result-service :request) :init)))
     (send req :goal_id (ros::uuid-to-unique-identifier-msg current-uuid))
     (ros::action-send-result-request name-space req))
   ;; Poll until result or timeout
   (let ((start (ros::time-now)))
     (while (and (null last-result) (ros::ok))
       (send self :spin-once)
       (unix::usleep 10000) ;; 10ms
       (when timeout
         (when (> (send (ros::time- (ros::time-now) start) :to-sec) timeout)
           (return-from :wait-for-result nil)))))
   (if last-result t nil))
  ;;
  (:send-goal-and-wait
   (goal &key (timeout nil))
   "Send a goal and wait for result."
   (send self :send-goal goal)
   (send self :wait-for-result :timeout timeout))
  ;;
  (:get-result () "Get the last received result." last-result)
  (:get-state () "Get the current goal state." current-goal-status)
  (:get-goal-status-string ()
   (case current-goal-status
     (0 "UNKNOWN") (1 "ACCEPTED") (2 "EXECUTING") (3 "CANCELING")
     (4 "SUCCEEDED") (5 "CANCELED") (6 "ABORTED")
     (t (format nil "INVALID(~A)" current-goal-status))))
  ;;
  (:cancel-goal
   ()
   "Cancel the current goal."
   (when current-uuid
     (let ((req (instance action_msgs::CancelGoalRequest :init)))
       (send req :goal_info
             (let ((gi (instance action_msgs::GoalInfo :init)))
               (send gi :goal_id
                     (ros::uuid-to-unique-identifier-msg current-uuid))
               gi))
       (ros::action-send-cancel name-space req))))
  ;;
  (:cancel-all-goals
   ()
   "Cancel all goals on this action server."
   (let ((req (instance action_msgs::CancelGoalRequest :init)))
     ;; Empty goal_info = cancel all
     (ros::action-send-cancel name-space req)))
  ;;
  (:name () name-space)
  (:destroy
   ()
   "Destroy the action client."
   (ros::action-client-destroy name-space))
  )

(defclass ros::simple-action-server
    ;; Simple Action Server for ROS 1 compatibility
  :super ros::object
  :slots (name-space action-spec action-type-str
          goal-class result-class feedback-class
          send-goal-service get-result-service feedback-message-class
          ;; Callbacks
          execute-cb preempt-cb
          ;; Current goal state
          current-goal current-goal-handle current-uuid preempt-requested
          is-active-flag
          ;; Result storage (server stores result, sends when client requests)
          pending-result pending-result-status result-ready
          groupname))

(defmethod ros::simple-action-server
  (:init
   (ns spec &key execute-cb preempt-cb ((:groupname gp)))
   "Initialize simple action server.
    ns: action namespace
    spec: action class
    execute-cb: callback when goal is received (funcall execute-cb goal)
    preempt-cb: callback when cancel is requested (funcall preempt-cb)"
   (setq name-space ns)
   (setq action-spec spec)
   (setq groupname gp)
   (setq action-type-str (get spec :datatype-))
   (setq goal-class (get spec :goal))
   (setq result-class (get spec :result))
   (setq feedback-class (get spec :feedback))
   (setq send-goal-service (get spec :send-goal-service))
   (setq get-result-service (get spec :get-result-service))
   (setq feedback-message-class (get spec :feedback-message))
   ;; Set callbacks
   (send self :set-execute-cb execute-cb)
   (send self :set-preempt-cb preempt-cb)
   ;; Initialize state
   (setq current-goal nil)
   (setq current-goal-handle nil)
   (setq current-uuid nil)
   (setq preempt-requested nil)
   (setq is-active-flag nil)
   (setq pending-result nil)
   (setq pending-result-status nil)
   (setq result-ready nil)
   ;; Initialize rcl_action server
   (ros::action-server-init name-space action-type-str)
   self)
  ;;
  (:set-execute-cb (cb) (setq execute-cb cb))
  (:set-preempt-cb (cb) (setq preempt-cb cb))
  ;;
  (:worker
   ()
   "Main server loop. Call this in your main loop.
    Polls for incoming goal requests, cancel requests, and result requests."
   ;; Process pending callbacks
   (if groupname (ros::spin-once groupname) (ros::spin-once))
   ;; Poll for new goal requests
   (let ((req (instance (get send-goal-service :request) :init)))
     (when (ros::action-take-goal-request name-space req)
       ;; ServerBase already accepted the goal and answered the client;
       ;; just pick up the handle it registered for us.
       (let ((uuid-bytes (send (send req :goal_id) :uuid)))
         (setq current-goal-handle
               (ros::action-accept-new-goal name-space uuid-bytes))
         (setq current-goal req) ;; store SendGoal_Request (like ROS 1 ActionGoal)
         (setq current-uuid uuid-bytes)
         (setq preempt-requested nil)
         (setq is-active-flag t)
         (setq result-ready nil)
         ;; NOTE: there is deliberately no result-request bookkeeping here.
         ;; The client sends its result request immediately after the goal
         ;; request and the two travel over separate services, so the result
         ;; request can reach the server first. rclcpp_action::ServerBase
         ;; handles that for us: it keys pending result requests on the goal
         ;; UUID, so :send-pending-result can just publish the result whenever
         ;; it is ready and ServerBase delivers it to whoever asked, whenever
         ;; they asked.
         ;; Publish status
         (ros::action-publish-status name-space)
         ;; Call execute callback (server goal) like ROS 1
         ;; goal is SendGoal_Request, use (send goal :goal) to get actual goal
         (when execute-cb
           (funcall execute-cb self current-goal)))))
   ;; Poll for cancel requests
   (let ((req (instance action_msgs::CancelGoalRequest :init)))
     (when (ros::action-take-cancel-request name-space req)
       ;; ServerBase already answered the cancel request.
       (setq preempt-requested t)
       (when preempt-cb
         (funcall preempt-cb)))))
  ;;
  (:goal () "Get the current goal." current-goal)
  (:goal-handle () "Get the current goal handle." current-goal-handle)
  ;;
  (:is-active ()
   "Check if a goal is currently active."
   is-active-flag)
  ;;
  (:is-preempt-requested ()
   "Check if preemption has been requested."
   preempt-requested)
  ;;
  (:send-pending-result
   ()
   "Internal: publish the result for the current goal.
    rclcpp_action::ServerBase matches the published result against the client's
    result request by goal UUID, answering a request that already arrived and
    storing the result for one that has not. So this no longer has to wait for
    the request to be observed first -- which used to be racy, because the goal
    and result services are separate DDS endpoints with no ordering guarantee
    between them."
   (when result-ready
     (let ((resp (instance (get get-result-service :response) :init)))
       (send resp :status pending-result-status)
       (when pending-result (send resp :result pending-result))
       (ros::action-send-result-response name-space current-uuid resp))
     (ros::action-notify-goal-done name-space)
     (setq result-ready nil)))
  ;;
  (:set-succeeded
   (result &optional (text ""))
   "Set the current goal as succeeded and send the result."
   (when (and current-goal-handle is-active-flag)
     ;; Update goal state to SUCCEEDED
     (ros::action-update-goal-state name-space current-goal-handle
                                    ros::*goal-event-succeed*)
     ;; Publish status
     (ros::action-publish-status name-space)
     ;; Store result (will be sent when client requests it)
     (setq pending-result result)
     (setq pending-result-status ros::*action-goal-state-succeeded*)
     (setq result-ready t)
     (setq is-active-flag nil)
     (send self :send-pending-result)))
  ;;
  (:set-aborted
   (result &optional (text ""))
   "Set the current goal as aborted."
   (when (and current-goal-handle is-active-flag)
     (ros::action-update-goal-state name-space current-goal-handle
                                    ros::*goal-event-abort*)
     (ros::action-publish-status name-space)
     (setq pending-result result)
     (setq pending-result-status ros::*action-goal-state-aborted*)
     (setq result-ready t)
     (setq is-active-flag nil)
     (send self :send-pending-result)))
  ;;
  (:set-preempted
   (&optional result (text ""))
   "Set the current goal as preempted (canceled)."
   (when (and current-goal-handle is-active-flag)
     ;; First transition to canceling, then to canceled
     (ros::action-update-goal-state name-space current-goal-handle
                                    ros::*goal-event-cancel-goal*)
     (ros::action-update-goal-state name-space current-goal-handle
                                    ros::*goal-event-canceled*)
     (ros::action-publish-status name-space)
     (setq pending-result result)
     (setq pending-result-status ros::*action-goal-state-canceled*)
     (setq result-ready t)
     (setq is-active-flag nil)
     (send self :send-pending-result)))
  ;;
  (:publish-feedback
   (feedback)
   "Publish feedback for the current goal."
   (when (and current-goal-handle is-active-flag current-uuid)
     (let ((fb-msg (instance feedback-message-class :init)))
       (send fb-msg :goal_id
             (ros::uuid-to-unique-identifier-msg current-uuid))
       (send fb-msg :feedback feedback)
       (ros::action-publish-feedback name-space fb-msg))))
  ;;
  (:result
   (&rest args)
   "Create a result instance."
   (let ((r (instance result-class :init)))
     (when args (send* r args))
     r))
  (:feedback
   (&rest args)
   "Create a feedback instance."
   (let ((f (instance feedback-class :init)))
     (when args (send* f args))
     f))
  ;;
  (:name () name-space)
  (:destroy
   ()
   "Destroy the action server."
   (ros::action-server-destroy name-space))
  )

(provide :actionlib2 "actionlib2.l")

(in-package "USER")
