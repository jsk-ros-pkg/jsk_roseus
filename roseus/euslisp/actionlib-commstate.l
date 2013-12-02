(if (not (find-package "ACTIONLIB_MSGS")) (ros::roseus-add-msgs "actionlib_msgs"))

(defun goal-status-to-string (i)
  (elt '(actionlib_msgs::GoalStatus::*pending* actionlib_msgs::GoalStatus::*active* actionlib_msgs::GoalStatus::*preempted* actionlib_msgs::GoalStatus::*succeeded* actionlib_msgs::GoalStatus::*aborted* actionlib_msgs::GoalStatus::*rejected* actionlib_msgs::GoalStatus::*preempting* actionlib_msgs::GoalStatus::*recalling* actionlib_msgs::GoalStatus::*recalled* actionlib_msgs::lost) i))

;; state machine transition
(setq
 ros::*comm-state-machine*
 (list
  (cons
   'ros::*comm-state-waiting-for-ack*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-pending*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-active*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-recalling*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-pending*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-active*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-recalling*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-active*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-waiting-for-result*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-invalid-transition*)))
  (cons
   'ros::*comm-state-waiting-for-cancel-ack*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-preempting*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-no-transition*)))
  (cons
   'ros::*comm-state-recalling*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-preempting*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-no-transition*)))
  (cons
   'ros::*comm-state-done*
   (list
    (cons actionlib_msgs::GoalStatus::*pending*    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*active*     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*rejected*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*recalling*  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::GoalStatus::*recalled*   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*preempted*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*succeeded*  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*aborted*    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::GoalStatus::*preempting* 'ros::*comm-state-invalid-transition*)))
  ))


(defclass actionlib-comm-state
  :super propertied-object
  :slots (state action-goal latest-goal-status latest-result))
(defmethod actionlib-comm-state
  (:init
   (&key ((:action-goal ac)) ((:action-result ar)) (name))
   (send self :name name)
   (setq state 'ros::*comm-state-waiting-for-ack*)
   (setq action-goal ac)
   (setq latest-goal-status (instance actionlib_msgs::GoalStatus :init :status actionlib_msgs::GoalStatus::*pending*))
   (setq latest-result ar))
  (:action-goal () action-goal)
  (:latest-goal-status () latest-goal-status)
  (:latest-result () latest-result)
  (:find-status-by-goal-id
   (msg)
   (let (ret)
     (ros::ros-debug "[~A] find-status-by-goal-id action-goal:~A" (send self :name) (if action-goal (send action-goal :goal_id :id)))
     (if action-goal
         (setq ret
               (find-if #'(lambda (m)
                            (ros::ros-debug "[~A] find-status-by-goal-id goal_id:~A == ~A ~A" (send self :name) (send m :goal_id :id) (send action-goal :goal_id :id) (string= (send m :goal_id :id) (send action-goal :goal_id :id)))
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
     (ros::ros-debug "[~A] COMM-STATE ~A~%    (~A -> ~A)~%    by status:~A goal_id:~A" (send self :name) comm-status state new-state (goal-status-to-string (send comm-status :status)) (send comm-status :goal_id :id))
     (setq latest-goal-status comm-status)
     (case new-state
           ('ros::*comm-state-no-transition*  t)
           ('ros::*comm-state-invalid-transition*
            (ros::ros-error (format nil "[~A] invalid goal status transition from ~A to ~A" (send self :name) (send comm-status :status) state)))
           (t
            (setq state new-state)))
     state))
  (:update-result
   (msg)
   (unless (and action-goal
                (string= (send msg :status :goal_id :id)
                         (send action-goal :goal_id :id)))
     (ros::ros-debug "[~A] invalid goal id: ~A ~A~%           msg :~A~%    action-goal:~A"
                    (send self :name) (send msg :status :goal_id :id)
                    (if action-goal (send action-goal :goal_id :id))
                    msg action-goal)
     (return-from :update-result nil))

   (setq latest-goal-status (send msg :status))
   (setq latest-result msg)
   (ros::ros-debug "[~A] update-result goal id: ~A ~A" (send self :name) (send msg :status :goal_id :id) state)
   (cond ((member state
                  (list 'ros::*comm-state-waiting-for-ack*
                        'ros::*comm-state-waiting-for-cancel-ack*
                        'ros::*comm-state-pending* 'ros::*comm-state-active*
                        'ros::*comm-state-waiting-for-result*
                        'ros::*comm-state-recalling*
                        'ros::*comm-state-preempting*))
          (send self :update-status
                (instance actionlib_msgs::GoalStatusArray :init :status_list (list (send msg :status))))
          (setq state 'ros::*comm-state-done*))
         ((eq state 'ros::*comm-state-done*)
          (ros::ros-error "[~A] Get a result when we ware already in the DONE state" (send self :name)))
         (t
          (ros::ros-error "[~A] In a funny state ~A" (send self :name) state)))
   (ros::ros-debug "[~A] update-result state: ~A ~A" (send self :name) (send msg :status :goal_id :id) state)
   )
  )

