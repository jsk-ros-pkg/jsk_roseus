#!/usr/bin/env roseus

(load "package://roseus_smach/src/state-machine-ros.l")
(load "package://roseus_smach/src/state-machine-utils.l")
(load "state-machine-sample.l")
;;
;; smach_viewer samples from SMACH tutorials
;;

(ros::roseus "smach_sample")

;; sample for
(ros::roseus-add-msgs "actionlib_tutorials")
(load "package://roseus_smach/src/state-machine-actionlib.l")
(defun smach-action-client-state ()
  (let* ((client (instance ros::simple-action-client :init "/fibonacci" actionlib_tutorials::FibonacciAction))
         (sm (instance state-machine :init)))
    (send sm :arg-keys :goal :cancel :result :feedback)

    (send sm :add-node
          (instance state :init :INITIAL
                    '(lambda (arg)
                      (let ((goal (instance actionlib_tutorials::FibonacciActionGoal :init)))
                        (send goal :goal :order 10)
                        (set-alist :goal goal arg))
                      (print 'initialized) t)
                    ))
    (send sm :add-node (actionlib-client-state :FIB client))
    (send sm :add-node
          (instance state :init :BAR
                    '(lambda (arg)
                      (warn "result: ~A~%" (cdr (assoc :result arg)))
                      (print 'succeded) t)
                    ))
    (send sm :start-state :INITIAL)
    (send sm :goal-state (list :SUCCEED-STATE :FAIL-STATE))

    ;; from and to nodes are selected by name or symbol
    (send sm :add-transition :INITIAL :FIB t)
    (send sm :add-transition :FIB :FAIL-STATE :failed)
    (send sm :add-transition :FIB :BAR :succeeded)
    (send sm :add-transition :BAR :SUCCEED-STATE t)

    sm ))

(defun exec-smach-simple () (setq count 0) (exec-state-machine (smach-simple)))
(defun exec-smach-nested () (setq count 0) (exec-state-machine (smach-nested)))
(defun exec-smach-userdata () (exec-state-machine (smach-userdata) '((count . 1))))

(warn ";;(exec-smach-simple)~%;;(exec-smach-nested)~%;;(exec-smach-userdata)~%")
