;; actionlib of roseus on ROS 2
;;
;; ros::simple-action-client and ros::simple-action-server with the API of
;; the ROS 1 roseus (actionlib.l), on top of the ROS 2 action protocol:
;;   <name>/_action/send_goal   (service  pkg/action/Foo_SendGoal)
;;   <name>/_action/get_result  (service  pkg/action/Foo_GetResult)
;;   <name>/_action/cancel_goal (service  action_msgs/srv/CancelGoal)
;;   <name>/_action/feedback    (topic    pkg/action/Foo_FeedbackMessage)
;;   <name>/_action/status      (topic    action_msgs/msg/GoalStatusArray)
;; so that roseus can talk to rclcpp_action / rclpy action clients and servers.
;;
;;  (instance ros::simple-action-client :init "fibonacci" action_tutorials_interfaces::Fibonacci)
;;  (instance ros::simple-action-server :init "fibonacci" action_tutorials_interfaces::Fibonacci
;;            :execute-cb #'execute-cb)
;;
;; The goal given to the :execute-cb of the server is pkg::Foo_SendGoalRequest
;; and (send goal :goal) is pkg::FooGoal, as the ActionGoal of ROS 1.
;; The states (:get-state) are the ones of ROS 1 actionlib_msgs/GoalStatus,
;; actionlib_msgs::GoalStatus::*succeeded* etc. are defined here.

(in-package "ROS")

(ros::roseus-add-msgs "action_msgs")
(ros::roseus-add-srvs "action_msgs")
(ros::roseus-add-msgs "unique_identifier_msgs")

;; the states of ROS 1 actionlib_msgs/GoalStatus
(unless (find-package "ACTIONLIB_MSGS") (make-package "ACTIONLIB_MSGS"))
(unless (find-package "ACTIONLIB_MSGS::GOALSTATUS") (make-package "ACTIONLIB_MSGS::GOALSTATUS"))
(dolist (s '(("*PENDING*" . 0) ("*ACTIVE*" . 1) ("*PREEMPTED*" . 2) ("*SUCCEEDED*" . 3)
             ("*ABORTED*" . 4) ("*REJECTED*" . 5) ("*PREEMPTING*" . 6) ("*RECALLING*" . 7)
             ("*RECALLED*" . 8) ("*LOST*" . 9)))
  (let ((sym (intern (car s) (find-package "ACTIONLIB_MSGS::GOALSTATUS"))))
    (unless (boundp sym) (set sym (cdr s)))))

(defun goal-status-to-string (i)
  (elt '(actionlib_msgs::GoalStatus::*pending* actionlib_msgs::GoalStatus::*active* actionlib_msgs::GoalStatus::*preempted* actionlib_msgs::GoalStatus::*succeeded* actionlib_msgs::GoalStatus::*aborted* actionlib_msgs::GoalStatus::*rejected* actionlib_msgs::GoalStatus::*preempting* actionlib_msgs::GoalStatus::*recalling* actionlib_msgs::GoalStatus::*recalled* actionlib_msgs::GoalStatus::*lost*) i))

(setq ros::*simple-goal-state-pending* 0
      ros::*simple-goal-state-active*  1
      ros::*simple-goal-state-done*    2)
(defun simple-goal-states-to-string (i)
  (elt '(ros::*simple-goal-state-pending* ros::*simple-goal-state-active* ros::*simple-goal-state-done*) i))

;; action_msgs/GoalStatus (ROS 2) <-> actionlib_msgs/GoalStatus (ROS 1)
(defun ros::action-status-to-ros1 (s)
  (case s
    (0 actionlib_msgs::GoalStatus::*pending*)    ;; UNKNOWN
    (1 actionlib_msgs::GoalStatus::*pending*)    ;; ACCEPTED
    (2 actionlib_msgs::GoalStatus::*active*)     ;; EXECUTING
    (3 actionlib_msgs::GoalStatus::*preempting*) ;; CANCELING
    (4 actionlib_msgs::GoalStatus::*succeeded*)  ;; SUCCEEDED
    (5 actionlib_msgs::GoalStatus::*preempted*)  ;; CANCELED
    (6 actionlib_msgs::GoalStatus::*aborted*)    ;; ABORTED
    (t actionlib_msgs::GoalStatus::*lost*)))
(defun ros::action-status-from-ros1 (s)
  (case s
    (0 1)  ;; PENDING -> ACCEPTED
    (1 2)  ;; ACTIVE -> EXECUTING
    (2 5)  ;; PREEMPTED -> CANCELED
    (3 4)  ;; SUCCEEDED
    (4 6)  ;; ABORTED
    (5 6)  ;; REJECTED -> ABORTED
    (6 3)  ;; PREEMPTING -> CANCELING
    (7 3)  ;; RECALLING -> CANCELING
    (8 5)  ;; RECALLED -> CANCELED
    (t 0)))
(defun ros::action-terminal-status-p (s) (member s '(4 5 6)))

(defvar ros::*action-uuid-count* 0)
(defun ros::make-uuid ()
  "16 bytes of a goal id (unique_identifier_msgs/UUID)"
  (let ((s (make-string 16))
        (now (ros::time-now))
        (pid (unix::getpid)))
    (incf ros::*action-uuid-count*)
    (dotimes (i 4) (setf (elt s i) (logand (ash (send now :nsec) (* -8 i)) #xff)))
    (dotimes (i 4) (setf (elt s (+ 4 i)) (logand (ash (send now :sec) (* -8 i)) #xff)))
    (dotimes (i 4) (setf (elt s (+ 8 i)) (logand (ash pid (* -8 i)) #xff)))
    (dotimes (i 2) (setf (elt s (+ 12 i)) (logand (ash ros::*action-uuid-count* (* -8 i)) #xff)))
    (dotimes (i 2) (setf (elt s (+ 14 i)) (random 256)))
    s))
(defun ros::uuid-to-string (uuid)
  (apply #'concatenate string (map cons #'(lambda (c) (format nil "~0,2x" c)) uuid)))
(defun ros::uuid-zero-p (uuid) (every #'zerop uuid))

(defun ros::action-topic (ns name)
  (format nil "~A/_action/~A" ns name))

;; for https://github.com/jsk-ros-pkg/jsk_roseus/pull/519
(unless (find-package "ROSACTIONLIB") (make-package "ROSACTIONLIB"))

(defclass ros::simple-action-client
  :super ros::object
  :slots (name-space action-spec simple-state goal-state goal-request goal-uuid
                     latest-result status-stamp
                     action-done-cb action-active-cb action-feedback-cb
                     groupname))

(defmethod ros::simple-action-client
  (:init
   (ns spec &key ((:groupname gp)))
   (send self :name ns)
   (setq name-space ns)
   (setq groupname gp)
   (if groupname (ros::create-nodehandle groupname))
   (setq action-spec spec)
   (unless (get spec :send-goal)
     (error "~A is not an action, use pkg::Foo or pkg::FooAction" spec))
   (if groupname
       (progn
         (ros::subscribe (ros::action-topic name-space "status") action_msgs::GoalStatusArray
                         #'send self :goal-status-cb 8 :groupname groupname)
         (ros::subscribe (ros::action-topic name-space "feedback") (get spec :feedback-message)
                         #'send self :action-feedback-cb 8 :groupname groupname))
     (progn
       (ros::subscribe (ros::action-topic name-space "status") action_msgs::GoalStatusArray
                       #'send self :goal-status-cb 8)
       (ros::subscribe (ros::action-topic name-space "feedback") (get spec :feedback-message)
                       #'send self :action-feedback-cb 8)))
   (setq simple-state ros::*simple-goal-state-done*)
   (setq goal-state actionlib_msgs::GoalStatus::*lost*)
   (set (gentemp "ACTIONLIBCLIENT" (find-package "ROSACTIONLIB")) self)
   self)
  ;;
  (:goal-status-cb
   (msg)
   (when goal-uuid
     (dolist (s (send msg :status_list))
       (when (string= (send s :goal_info :goal_id :uuid) goal-uuid)
         (setq status-stamp (ros::time-now))
         ;; the result is given by get_result
         (unless (or (= simple-state ros::*simple-goal-state-done*)
                     (ros::action-terminal-status-p (send s :status)))
           (setq goal-state (ros::action-status-to-ros1 (send s :status)))
           (when (and (= simple-state ros::*simple-goal-state-pending*)
                      (member (send s :status) '(2 3)))
             (setq simple-state ros::*simple-goal-state-active*)))))))
  (:action-feedback-cb
   (msg)
   (when (and goal-uuid (string= (send msg :goal_id :uuid) goal-uuid))
     (ros::ros-debug "[~A] feedback-cb ~A" name-space msg)
     (if action-feedback-cb (funcall action-feedback-cb msg))))
  (:goal-response-cb
   (uuid res)
   (when (and goal-uuid (string= uuid goal-uuid))
     (cond
      ((send res :accepted)
       (ros::ros-debug "[~A] goal ~A accepted" name-space (ros::uuid-to-string uuid))
       (when (= goal-state actionlib_msgs::GoalStatus::*lost*)
         (setq goal-state actionlib_msgs::GoalStatus::*pending*))
       (if action-active-cb (funcall action-active-cb))
       (let ((req (instance (get (get action-spec :get-result) :request) :init)))
         (send req :goal_id :uuid uuid)
         (ros::service-call-async (ros::action-topic name-space "get_result") req
                                  #'send self :result-cb uuid)))
      (t
       (ros::ros-warn "[~A] goal ~A rejected" name-space (ros::uuid-to-string uuid))
       (setq goal-state actionlib_msgs::GoalStatus::*rejected*)
       (setq latest-result (instance (get (get action-spec :get-result) :response) :init))
       (setq simple-state ros::*simple-goal-state-done*)
       (if action-done-cb (funcall action-done-cb goal-state (send latest-result :result)))))))
  (:result-cb
   (uuid res)
   (when (and goal-uuid (string= uuid goal-uuid))
     (ros::ros-debug "[~A] result-cb ~A ~A" name-space (ros::uuid-to-string uuid) (send res :status))
     (setq latest-result res)
     (setq goal-state (ros::action-status-to-ros1 (send res :status)))
     (if (= (send res :status) 0) (setq goal-state actionlib_msgs::GoalStatus::*lost*))
     (setq simple-state ros::*simple-goal-state-done*)
     (if action-done-cb (funcall action-done-cb goal-state (send res :result)))))
  ;;
  (:wait-for-server
   (&optional (timeout nil))
   (let ((count 0))
     (ros::rate 10)
     (while (not (and (ros::service-exists (ros::action-topic name-space "send_goal"))
                      (ros::service-exists (ros::action-topic name-space "get_result"))
                      (ros::service-exists (ros::action-topic name-space "cancel_goal"))))
       (send self :spin-once)
       (ros::sleep)
       (when (and (> count 0) (= 0 (mod count 50)))
         (ros::ros-warn "Still waiting for [~A] action server. ~A seconds have passed." name-space (/ count 10)))
       (when (and timeout (>= count (* timeout 10)))
         (ros::ros-warn "[~A] action server is not found" name-space)
         (return-from :wait-for-server nil))
       (unless (ros::ok) (return-from :wait-for-server nil))
       (incf count))
     t))
  (:make-goal-instance
   (&rest args)
   (let ((msg (instance (get (get action-spec :send-goal) :request) :init)))
     (send* msg :goal args)
     msg))
  (:send-goal
   (goal &key done-cb active-cb feedback-cb)
   ;; accepts both pkg::FooGoal and pkg::Foo_SendGoalRequest (ActionGoal)
   (unless (assoc 'ros::_goal_id (send goal :slots))
     (setq goal (send self :make-goal-instance goal)))
   (setq goal-uuid (ros::make-uuid))
   (send goal :goal_id :uuid goal-uuid)
   (setq goal-request goal)
   (setq simple-state ros::*simple-goal-state-pending*)
   (setq goal-state actionlib_msgs::GoalStatus::*lost*)
   (setq latest-result nil status-stamp nil)
   (setq action-done-cb done-cb action-active-cb active-cb action-feedback-cb feedback-cb)
   (ros::ros-debug "[~A] send-goal ~A" name-space (ros::uuid-to-string goal-uuid))
   (unless (ros::service-call-async (ros::action-topic name-space "send_goal") goal
                                    #'send self :goal-response-cb goal-uuid)
     (ros::ros-error "[~A] failed to send goal" name-space)
     (setq simple-state ros::*simple-goal-state-done*))
   goal)
  (:send-goal-and-wait
   (goal &key (timeout 0))
   (send self :send-goal goal)
   (unless (send self :wait-for-result :timeout timeout)
     (send self :cancel-goal)
     (send self :wait-for-result :timeout timeout)))
  (:wait-for-result
   (&key (timeout 0) (return-if-server-down) (maximum-status-interval 5) (wait-rate 100))
   (let ((start (ros::time-now)))
     (unless goal-uuid
       (ros::ros-error "[~A] :wait-for-result (return nil when no goal exists)" name-space)
       (return-from :wait-for-result nil))
     (ros::rate wait-rate)
     (while (ros::ok)
       (send self :spin-once)
       (if (= simple-state ros::*simple-goal-state-done*) (return))
       (if (and (> timeout 0) (> (send (ros::time- (ros::time-now) start) :to-sec) timeout))
           (return-from :wait-for-result nil))
       (when (and return-if-server-down
                  (not (ros::service-exists (ros::action-topic name-space "get_result"))))
         (ros::ros-error "[~A] Unexpected returns from :wait-for-result : the action server is down" name-space)
         (return-from :wait-for-result nil))
       (ros::sleep))
     (if (eq (send self :get-state) actionlib_msgs::GoalStatus::*preempted*)
         (ros::ros-warn "[~A] :wait-for-result finished with preempted status" name-space))
     (eq (send self :get-state) actionlib_msgs::GoalStatus::*succeeded*)))
  (:get-result () (if latest-result (send latest-result :result)))
  (:get-state ()
   (cond
    ((= goal-state actionlib_msgs::GoalStatus::*recalling*) actionlib_msgs::GoalStatus::*pending*)
    ((= goal-state actionlib_msgs::GoalStatus::*preempting*) actionlib_msgs::GoalStatus::*active*)
    (t goal-state)))
  (:get-goal-status-text () (goal-status-to-string (send self :get-state)))
  (:cancel-request (uuid)
   (let ((req (instance action_msgs::CancelGoalRequest :init)))
     (send req :goal_info :goal_id :uuid uuid)
     (ros::service-call-async (ros::action-topic name-space "cancel_goal") req
                              #'(lambda (res)
                                  (ros::ros-debug "cancel_goal returns ~A" (send res :return_code))))))
  (:cancel-all-goals () (send self :cancel-request (make-string 16)))
  (:cancel-goal ()
   (when goal-uuid
     (send self :cancel-request goal-uuid)
     (unless (= simple-state ros::*simple-goal-state-done*)
       (setq goal-state actionlib_msgs::GoalStatus::*preempting*))))
  ;;
  (:name-space () name-space)
  (:spin-once ()
   (if groupname (ros::spin-once groupname) (ros::spin-once)))
  )

(defclass ros::simple-action-server
  :super ros::object
  :slots (name-space action-spec status
                     execute-cb accept-cb preempt-cb
                     goal goal-uuid goal-stamp pending-goal
                     results waiting-results status-published-time
                     groupname))

(defmethod ros::simple-action-server
  (:init
   (ns spec &key ((:execute-cb exec-f)) ((:preempt-cb preempt-f))
       ((:groupname gp)) ((:accept-cb accept-f)))
   (send self :name ns)
   (setq name-space ns)
   (setq action-spec spec)
   (unless (get spec :send-goal)
     (error "~A is not an action, use pkg::Foo or pkg::FooAction" spec))
   (setq execute-cb exec-f preempt-cb preempt-f accept-cb accept-f)
   (setq status actionlib_msgs::GoalStatus::*pending*)
   (setq groupname gp)
   (if groupname (ros::create-nodehandle groupname))
   (ros::advertise (ros::action-topic name-space "status") action_msgs::GoalStatusArray 1 t)
   (ros::advertise (ros::action-topic name-space "feedback") (get spec :feedback-message) 10)
   (let ((args (if groupname (list :groupname groupname))))
     (apply #'ros::advertise-service (ros::action-topic name-space "send_goal") (get spec :send-goal)
            #'send self :send-goal-cb args)
     (apply #'ros::advertise-service (ros::action-topic name-space "get_result") (get spec :get-result)
            #'send self :get-result-cb args)
     (apply #'ros::advertise-service (ros::action-topic name-space "cancel_goal") action_msgs::CancelGoal
            #'send self :cancel-cb args))
   (send self :publish-status)
   self)
  ;;
  (:status-list ()
   (let (l)
     (dolist (r results)
       (push (instance action_msgs::GoalStatus :init
                       :goal_info (instance action_msgs::GoalInfo :init
                                            :goal_id (instance unique_identifier_msgs::UUID :init :uuid (car r))
                                            :stamp (cadr r))
                       :status (send (caddr r) :status))
             l))
     (when goal
       (push (instance action_msgs::GoalStatus :init
                       :goal_info (instance action_msgs::GoalInfo :init
                                            :goal_id (send goal :goal_id) :stamp goal-stamp)
                       :status (ros::action-status-from-ros1 status))
             l))
     (when pending-goal
       (push (instance action_msgs::GoalStatus :init
                       :goal_info (instance action_msgs::GoalInfo :init
                                            :goal_id (send (car pending-goal) :goal_id)
                                            :stamp (cdr pending-goal))
                       :status 1)
             l))
     l))
  (:publish-status ()
   (setq status-published-time (ros::time-now))
   (ros::publish (ros::action-topic name-space "status")
                 (instance action_msgs::GoalStatusArray :init :status_list (send self :status-list))))
  (:execute-cb
   ()
   (if (and goal execute-cb) (funcall execute-cb self goal))
   ;; ROS 1 publishes status at every :worker, here once per second or when changed
   (when (or (null status-published-time)
             (> (send (ros::time- (ros::time-now) status-published-time) :to-sec) 1.0))
     (send self :publish-status)))
  (:start-goal (req stamp)
   (setq status actionlib_msgs::GoalStatus::*active*)
   (setq goal req goal-uuid (send req :goal_id :uuid) goal-stamp stamp)
   (if accept-cb (funcall accept-cb self goal)))
  (:send-goal-cb ;; goal is always accepted, not rejected
   (req)
   (let ((stamp (ros::time-now)))
     (ros::ros-info ";; Received Goal ~A" (ros::uuid-to-string (send req :goal_id :uuid)))
     (cond (pending-goal ;; recall the pending goal
            (send self :store-result (send (car pending-goal) :goal_id :uuid) (cdr pending-goal)
                  (let ((r (send self :result))) (send r :status 5) r))
            (setq pending-goal (cons req stamp)))
           ((send self :is-active)
            (setq status actionlib_msgs::GoalStatus::*preempting*)
            (if preempt-cb (funcall preempt-cb self goal))
            (setq pending-goal (cons req stamp)))
           (t
            (send self :start-goal req stamp)))
     (send self :publish-status)
     (let ((res (send req :response)))
       (send res :accepted t)
       (send res :stamp stamp)
       res)))
  (:get-result-cb
   (req)
   (let* ((uuid (send req :goal_id :uuid))
          (r (assoc uuid results :test #'string=)))
     (cond
      (r (caddr r))
      ((or (and goal (string= uuid goal-uuid))
           (and pending-goal (string= uuid (send (car pending-goal) :goal_id :uuid))))
       (push (cons uuid (ros::service-request-handle)) waiting-results)
       :deferred)
      (t ;; unknown goal
       (let ((res (send req :response))) (send res :status 0) res)))))
  (:cancel-cb
   (req)
   (let* ((info (send req :goal_info))
          (uuid (send info :goal_id :uuid))
          (stamp (send info :stamp))
          (all (ros::uuid-zero-p uuid))
          (res (send req :response))
          canceling)
     (ros::ros-info ";; Received Cancel")
     (flet ((match (u st)
              (or (and (not all) (string= uuid u))
                  (and all (or (= 0 (send stamp :to-sec)) (not (ros::time< stamp st))))
                  (and (not (ros::uuid-zero-p uuid)) nil))))
       (when (and pending-goal (match (send (car pending-goal) :goal_id :uuid) (cdr pending-goal)))
         (push (instance action_msgs::GoalInfo :init :goal_id (send (car pending-goal) :goal_id)
                         :stamp (cdr pending-goal)) canceling)
         (send self :store-result (send (car pending-goal) :goal_id :uuid) (cdr pending-goal)
               (let ((r (send self :result))) (send r :status 5) r))
         (setq pending-goal nil))
       (when (and goal (send self :is-active) (match goal-uuid goal-stamp))
         (push (instance action_msgs::GoalInfo :init :goal_id (send goal :goal_id) :stamp goal-stamp)
               canceling)
         (setq status actionlib_msgs::GoalStatus::*preempting*)
         (if preempt-cb (funcall preempt-cb self goal))))
     (send res :goals_canceling canceling)
     (send res :return_code (if canceling 0 2)) ;; ERROR_NONE / ERROR_UNKNOWN_GOAL_ID
     (send self :publish-status)
     res))
  ;;
  (:store-result (uuid stamp res)
   (push (list uuid stamp res) results)
   ;; keep 100 finished goals for late get_result requests
   (if (> (length results) 100) (setq results (subseq results 0 100)))
   (dolist (w (remove-if-not #'(lambda (w) (string= (car w) uuid)) waiting-results))
     (ros::send-service-response (cdr w) res))
   (setq waiting-results (remove-if #'(lambda (w) (string= (car w) uuid)) waiting-results)))
  (:publish-result
   (msg &optional (text ""))
   (ros::ros-info ";; Send Result ~A" (ros::uuid-to-string goal-uuid))
   (send msg :status (ros::action-status-from-ros1 status))
   (send self :store-result goal-uuid goal-stamp msg)
   (setq status actionlib_msgs::GoalStatus::*pending*)
   (let ((next-goal pending-goal))
     (setq goal nil goal-uuid nil pending-goal nil)
     (if next-goal (send self :start-goal (car next-goal) (cdr next-goal))))
   (send self :publish-status))
  (:publish-feedback
   (msg)
   (send msg :goal_id :uuid goal-uuid)
   (ros::publish (ros::action-topic name-space "feedback") msg))
  ;;
  (:set-succeeded
   (msg &optional (text ""))
   (ros::ros-info ";; Action Succeeded ~A" (ros::uuid-to-string goal-uuid))
   (setq status actionlib_msgs::GoalStatus::*succeeded*)
   (send self :publish-result msg text))
  (:set-aborted
   (msg &optional (text ""))
   (ros::ros-info ";; Action Aborted ~A" (ros::uuid-to-string goal-uuid))
   (setq status actionlib_msgs::GoalStatus::*aborted*)
   (send self :publish-result msg text))
  (:set-preempted
   (&optional (msg (send self :result)) (text ""))
   (ros::ros-info ";; Cancel Goal ~A" (ros::uuid-to-string goal-uuid))
   (setq status actionlib_msgs::GoalStatus::*preempted*)
   (send self :publish-result msg text))
  ;;
  (:goal () goal)
  (:result
   (&rest args)
   (let ((msg (instance (get (get action-spec :get-result) :response) :init)))
     (send* msg :result args)
     msg))
  (:feedback
   (&rest args)
   (let ((msg (instance (get action-spec :feedback-message) :init)))
     (send* msg :feedback args)
     msg))
  (:worker () (send self :execute-cb))
  (:is-preempt-requested () (eq status actionlib_msgs::GoalStatus::*preempting*))
  (:is-active () (memq status (list actionlib_msgs::GoalStatus::*preempting* actionlib_msgs::GoalStatus::*active*)))
  (:name-space () name-space)
  (:spin-once ()
   (if groupname (ros::spin-once groupname) (ros::spin-once)))
  )

(provide :actionlib "actionlib.l")
