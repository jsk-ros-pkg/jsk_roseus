(if (not (find-package "ACTIONLIB_MSGS")) (ros::roseus-add-msgs "actionlib_msgs"))

(defun goal-status-to-string (i)
  (elt '(actionlib_msgs::*pending* actionlib_msgs::*active* actionlib_msgs::*preempted* actionlib_msgs::*succeeded* actionlib_msgs::*aborted* actionlib_msgs::*rejected* actionlib_msgs::*preempting* actionlib_msgs::*recalling* actionlib_msgs::*recalled* actionlib_msgs::lost) i))

;; state machine transition
(setq
 ros::*comm-state-machine*
 (list
  (cons
   'ros::*comm-state-waiting-for-ack*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-pending*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-active*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-recalling*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-pending*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-active*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-recalling*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-active*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-waiting-for-result*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-invalid-transition*)))
  (cons
   'ros::*comm-state-waiting-for-cancel-ack*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-preempting*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-no-transition*)))
  (cons
   'ros::*comm-state-recalling*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-preempting*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-no-transition*)))
  (cons
   'ros::*comm-state-done*
   (list
    (cons actionlib_msgs::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*active*     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*rejected*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::*recalled*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*preempted*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*succeeded*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*aborted*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::*preempting* 'ros::*comm-state-invalid-transition*)))
  ))


(defclass actionlib-comm-state
  :super propertied-object
  :slots (state action-goal latest-goal-status latest-result))
(defmethod actionlib-comm-state
  (:init
   (&key ((:action-goal ac)))
   (setq state 'ros::*comm-state-waiting-for-ack*)
   (setq action-goal ac)
   (setq latest-goal-status (instance actionlib_msgs::GoalStatus :init :status actionlib_msgs::*pending*))
   (setq latest-result nil))
  (:action-goal () action-goal)
  (:latest-goal-status () latest-goal-status)
  (:latest-result () latest-result)
  (:find-status-by-goal-id
   (msg)
   (let (ret)
     (if action-goal
         (setq ret
               (find-if #'(lambda (m)
                            (string= (send m :goal_id :id)
                                     (send action-goal :goal_id :id)))
                        (send msg :status_list))))
     ret))
  (:state (&optional s) (if s (setq state s)) state)
  (:update-status
   (msg)
   (let (comm-status new-state)
     (if (eq state 'ros::*comm-state-done*) (return-from :update-status state))
     (setq comm-status (send self :find-status-by-goal-id msg))
     (unless comm-status (return-from :update-status state))
     (setq new-state
           (cdr (assoc (send comm-status :status)
                       (cdr (assoc state ros::*comm-state-machine*)))))
     (ros::ros-debug "COMM-STATE ~A~%    (~A -> ~A)~%    by status:~A goal_id:~A~%" comm-status state new-state (goal-status-to-string (send comm-status :status)) (send comm-status :goal_id :id))
     (setq latest-goal-status comm-status)
     (case new-state
           ('ros::*comm-state-no-transition*  t)
           ('ros::*comm-state-invalid-transition*
            (ros::ros-error (format nil "invalid goal status tronasition from ~A to ~A" (quote s) (quote new-state))))
           (t
            (setq state new-state)))
     state))
  (:update-result
   (msg)
   (unless (and action-goal
		(string= (send msg :status :goal_id :id)
			 (send action-goal :goal_id :id)))
     (return-from :update-result))

   (setq latest-goal-status (send msg :status))
   (setq latest-result msg)
   (cond ((member state
                  (list 'ros::*comm-state-waiting-for-ack*
                        'ros::*comm-state-pending* 'ros::*comm-state-active*
                        'ros::*comm-state-waiting-for-result*
                        'ros::*comm-state-recalling*
                        'ros::*comm-state-preempting*))
          (send self :update-status
                (instance actionlib_msgs::GoalStatusArray :init :status_list (list (send msg :status))))
          (setq state 'ros::*comm-state-done*))
         ((eq state 'ros::*comm-state-done*)
          (ros::ros-error "Get a result when we ware already in the DONE state"))
         (t
          (ros::ros-error (format nil "In a fanny state ~A" state))))
   )
  )

