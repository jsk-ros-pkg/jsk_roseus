(if (not (find-package "ACTIONLIB_MSGS")) (ros::roseus-add-msgs "actionlib_msgs"))

(defun goal-status-to-string (i)
  (elt '(actionlib_msgs::pending actionlib_msgs::active actionlib_msgs::rejected actionlib_msgs::recalling actionlib_msgs::recalled actionlib_msgs::preempted actionlib_msgs::succeeded actionlib_msgs::aborted actionlib_msgs::preempting) i))

;; state machine transition
(setq
 ros::*comm-state-machine*
 (list
  (cons
   'ros::*comm-state-waiting-for-ack*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-pending*)
    (cons actionlib_msgs::active     'ros::*comm-state-active*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-recalling*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-pending*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::active     'ros::*comm-state-active*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-recalling*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-active*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::active     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-waiting-for-result*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::active     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-invalid-transition*)))
  (cons
   'ros::*comm-state-waiting-for-cancel-ack*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::active     'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-preempting*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-no-transition*)))
  (cons
   'ros::*comm-state-recalling*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::active     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-preempting*)))
  (cons
   'ros::*comm-state-preempting*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::active     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-waiting-for-result*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-no-transition*)))
  (cons
   'ros::*comm-state-done*
   (list
    (cons actionlib_msgs::pending    'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::active     'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::rejected   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::recalling  'ros::*comm-state-invalid-transition*)
    (cons actionlib_msgs::recalled   'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::preempted  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::succeeded  'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::aborted    'ros::*comm-state-no-transition*)
    (cons actionlib_msgs::preempting 'ros::*comm-state-invalid-transition*)))
  ))


(defclass actionlib-comm-state
  :super propertied-object
  :slots (state))
(defmethod actionlib-comm-state
  (:init
   ()
   (setq state 'ros::*comm-state-waiting-for-ack*))
  (:state (&optional s) (if s (setq state s)) state)
  (:update-state
   (s)
   (let (tmp-state)
     (if (eq state 'ros::*comm-state-done*) (return-from :update-state state))
     (setq tmp-state
           (cdr (assoc s (cdr (assoc state ros::*comm-state-machine*)))))
     ;;(warning-message 1 "COMM-STATE ~A -> ~A~%    by ~A~%" state tmp-state (goal-status-to-string s))
     (case tmp-state
           ('ros::*comm-state-no-transition*  t)
           ('ros::*comm-state-invalid-transition*
            (ros::ros-error (format nil "invalid goal status tronasition from ~A to ~A" (quote s) (quote tmp-state))))
           (t
            (setq state tmp-state)))
     state))
  )