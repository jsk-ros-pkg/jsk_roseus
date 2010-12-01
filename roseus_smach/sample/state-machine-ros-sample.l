(load "package://roseus_smach/src/state-machine-ros.l")
(load "state-machine-sample.l")
;;
;; smach_viewer samples from SMACH tutorials
;;

(ros::roseus "smach_sample")

;; execute state-machine and publish every step
(defun smach-exec (sm)
  (let ((insp (instance state-machine-inspector :init sm))
	(mydata (list (cons 'count 0) (cons 'hoge (list 1 2 3)))))
    (unix:sleep 1) ;; for advertize topics successfully
    (send sm :reset-state)
    (send insp :publish-structure)
    (send insp :publish-status mydata)
    (while (not (send sm :goal-reached))
      (unix:sleep 1)
      (send insp :publish-status mydata)
      ;; you can execute state-machine step by step
      (send sm :execute mydata :step -1))
    (send sm :active-state) ;; return reached goal state
    ))

(defun smach-exec-simple () (setq count 0) (smach-exec (smach-simple)))
(defun smach-exec-nested () (setq count 0) (smach-exec (smach-nested)))
(defun smach-exec-userdata () (smach-exec (smach-userdata)))

(warn ";;(smach-exec-simple)~%;;(smach-exec-nested)~%;;(smach-exec-userdata)~%")
