#!/usr/bin/env roseus
;;;
;;; Fibonacci action server with step-by-step execution
;;; Supports cooperative cancellation and publishes feedback per step.
;;;
(ros::load-ros-manifest "roseus")
(require :actionlib2 "actionlib2.l")

(ros::roseus "fibonacci")
(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

;; Step-by-step execution state
(setq *order* nil)
(setq *sequence* nil)
(setq *step* 0)

;; No execute-cb: handle goals manually in main loop
(setq *s* (instance ros::simple-action-server :init
                    "fibonacci" roseus::Fibonacci))

(ros::rate 200)
(do-until-key
  (send *s* :worker)
  ;; Process active goal step by step
  (when (and *order* (send *s* :is-active))
    (cond
      ;; Cancel requested
      ((send *s* :is-preempt-requested)
       (warning-message 2 ";; cancel requested at step ~A~%" *step*)
       (send *s* :set-preempted
             (send *s* :result :sequence (nreverse *sequence*)))
       (setq *order* nil))
      ;; Computation not done yet
      ((< *step* (1- *order*))
       (push (+ (car *sequence*) (cadr *sequence*)) *sequence*)
       (send *s* :publish-feedback
             (send *s* :feedback :sequence (reverse *sequence*)))
       (incf *step*)
       ;; Slow down to give client time to cancel
       (unix::usleep 10000))
      ;; Computation done
      (t
       (send *s* :set-succeeded
             (send *s* :result :sequence (nreverse *sequence*)))
       (setq *order* nil))))
  ;; Check for new goals (when idle)
  ;; Use :is-active (set by :worker when accepting a new goal, cleared by set-succeeded)
  ;; instead of :goal (which retains old goal after completion)
  (when (and (null *order*) (send *s* :is-active))
    (let ((goal (send *s* :goal)))
      (setq *order* (send goal :goal :order))
      (setq *sequence* (list 1 0))
      (setq *step* 0)
      (warning-message 2 ";; new goal: order=~A~%" *order*)
      ;; Transition to executing
      (ros::action-update-goal-state
        (send *s* :name)
        (send *s* :goal-handle)
        ros::*goal-event-execute*)
      (ros::action-publish-status (send *s* :name))))
  (ros::sleep))
