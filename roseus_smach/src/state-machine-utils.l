;; state-machine-utils.l

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

(defun get-alist (key alist)
  (if (assoc key alist)
    (cadr (assoc key alist)) nil))

(defun remove-seq (subseq seq &key (test #'eq))
  (if subseq (remove-seq (cdr subseq) (remove (car subseq) seq :test test) :test test) seq))

(defun make-state-machine (graph-list func-map initial-state goal-states &key (parallel-exec-result t))
  (let ((sm (instance state-machine :init))
        (all-nodes (unique
                    (flatten
                     (mapcar #'(lambda (connection)
                                 (append (list (car connection))
                                         (list (caddr connection))))
                             graph-list)))))
    (warning-message 2 "~A nodes are defined~%" (length all-nodes))

    ;; add all action states to state machine
    (dolist (node (remove-seq goal-states all-nodes))
      (warning-message 2 "mapped function: ~A -> ~A~%" node (eval (get-alist node func-map)))
      (send sm :add-node (instance state :init node
                                   (eval (get-alist node func-map)))))

    ;; add goal states
    (send sm :goal-state goal-states)

    ;; create connection between nodes
    (dolist (connection graph-list)
      (let ((from-nodes (flatten (list (car connection))))
            (to-nodes (flatten (list (caddr connection)))))
        (warning-message 2 "~A -> ~A~%" from-nodes to-nodes)
        (dolist (from-node from-nodes)
          (dolist (to-node to-nodes)
            (send sm :add-transition
                  (send sm :node from-node)
                  (send sm :node to-node)
                  (case (length to-nodes)
                    (1 t)
                    (t parallel-exec-result)))))))
    (send sm :parallel-exec-result parallel-exec-result)

    ;; set initial states
    (send sm :start-state initial-state)
    sm))

(provide :state-machine-utils) ;; end of state-machine-utils.l
