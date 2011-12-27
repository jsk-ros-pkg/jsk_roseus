;; execute the state-machine with spining
;; when it occures an error, stop and return the initial state
;; the loop is do-until-key type
(defun exec-smach-with-spin (sm &optional mydata &key (hz 1))
  (let ((insp (instance state-machine-inspector :init sm)))
    (unix::sleep 2)
    (send sm :reset-state)
    (send insp :publish-structure) ;; publish once and latch

    (unwind-protect
     ;; try block
     (do-until-key
      (when (not (ros::ok)) (return))
      (ros::spin-once)
      (send insp :publish-status mydata)
      (when (not (send sm :goal-reached))
	(send sm :execute mydata :step -1))
      (unix::usleep (round (/ 1e6 hz))))
     ;; finally block
     (progn
       (send sm :reset-state)
       (print "exit (exec-smach-with-spin)"))
     )
    ))
