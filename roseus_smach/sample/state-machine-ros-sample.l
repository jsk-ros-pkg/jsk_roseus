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
      (ros::spin-once)
      (send insp :publish-status mydata)
      ;; you can execute state-machine step by step
      (send sm :execute mydata :step -1))
    (send sm :active-state) ;; return reached goal state
    ))

;; sample for 
(ros::roseus-add-msgs "actionlib_tutorials")
(load "package://roseus_smach/src/state-machine-actionlib.l")
(defun smach-action-server ()
  (let* ((client (instance ros::simple-action-client :init "/fibonacci" actionlib_tutorials::FibonacciAction))
	 (sm (instance state-machine :init)))
    (send sm :arg-keys :goal :cancel)

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

(defun smach-exec-simple () (setq count 0) (smach-exec (smach-simple)))
(defun smach-exec-nested () (setq count 0) (smach-exec (smach-nested)))
(defun smach-exec-userdata () (smach-exec (smach-userdata)))

(warn ";;(smach-exec-simple)~%;;(smach-exec-nested)~%;;(smach-exec-userdata)~%")
