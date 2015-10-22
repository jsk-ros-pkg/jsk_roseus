;; state-machine-utils.l

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

;; execute the state-machine with spining
;; when it occures an error, stop and return the initial state
;; the loop is do-until-key type
(defun exec-smach-with-spin (sm &optional mydata &key (hz 1) iterate)
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
       (when iterate (read-char))
      (unix::usleep (round (/ 1e6 hz))))
     ;; finally block
     (progn
       (send sm :reset-state)
       (print "exit (exec-smach-with-spin)"))
     )
    ))

(defun make-state-machine (graph-list func-map initial-state goal-states
                           &key (exec-result t) (exec-failure-result nil) (parallel-exec-result t))
  "This is utility function to make state machine
e.g. (make-state-machine '((:a -> :b)
                           (:b -> :c)
                           (:b !-> :b_f)
                           (:b_f -> :fail)
                           (:c -> (:d1 :d2))
                           (:d1 -> :end)
                           (:d2 -> :end))
                         '((:a #'action-a)
                           (:b #'action-b)
                           (:c #'action-c)
                           (:d1 #'action-d1)
                           (:d2 #'action-d2))
                          '(:a)
                          '(:end :fail))
    This makes state machine like:
      :a -> :b -> :c -> :d1 -> :end
             |  t  \ -> :d2 /
         nil  -> :b_f -> :fail
    Each actions are assumed to implement like below:
    ```
    (defun action-a (&rest args)
     (if (eq (do-action-a) :success)
       t nil))
    ```
    By default, if `action-a` returns value t, then state transits as defined above.
    You can specify return value as `exec-result` key.
"
  (labels ((unique-graph (gr)
             (unique
              (flatten
               (mapcar #'(lambda (conn)
                           (append (list (car conn))
                                   (list (caddr conn)))) gr))))
           (get-alist (key alist)
                      (if (assoc key alist)
                          (cadr (assoc key alist)) nil)))
    (let ((sm (instance state-machine :init))
          (all-nodes (unique-graph graph-list)))
      (warning-message 2 "~A nodes are defined~%" (length all-nodes))

      ;; add all action states to state machine
      (dolist (node (set-difference all-nodes goal-states))
        (cond
         ((functionp (eval (get-alist node func-map)))
          (warning-message 2 "mapped function: ~A -> ~A~%" node (eval (get-alist node func-map)))
          (send sm :add-node (instance state :init node
                                       (eval (get-alist node func-map)))))
         ((subclassp (class (eval (get-alist node func-map))) state-machine)
          (warning-message 2 "mapped sub state machine: ~A -> ~A~%" node (eval (get-alist node func-map)))
          (send sm :add-node (instance state :init node
                                       (eval (get-alist node func-map)))))
         ((subclassp (class (eval (get-alist node func-map))) state)
          (unless (eq (send (eval (get-alist node func-map)) :name) node)
            (error "node name(~A) and name of given state(~A) must be same!~%"
                   node (send (eval (get-alist node func-map) :name))))
          (warning-message 2 "mapped state passthrough: ^A -> ~A" node (eval (get-alist node func-map)))
          (send sm :add-node (eval (get-alist node func-map))))
         (t
          (error "illegal type of func-map element: ~A~%" (eval (get-alist node func-map))))))

      ;; add goal states
      (send sm :goal-state goal-states)

      ;; set parallel execution result
      (send sm :parallel-exec-result parallel-exec-result)

      ;; set initial states
      (send sm :start-state initial-state)

      ;; create connection between nodes
      (dolist (connection graph-list)
        (let ((from-nodes (flatten (list (car connection))))
              (edge (cadr connection))
              (to-nodes (flatten (list (caddr connection)))))
              (warning-message 2 "~A ~A ~A~%" from-nodes edge to-nodes)
          (dolist (from-node from-nodes)
            (dolist (to-node to-nodes)
              (send sm :add-transition
                    (send sm :node from-node)
                    (send sm :node to-node)
                    (cond
                     ((> (length to-nodes) 1) parallel-exec-result)
                     ((eq edge '->) exec-result)
                     ((eq edge '!->) exec-failure-result)
                     (t (error "invalid connection ~A ~A ~A" from-node edge to-node))))))))
      sm)))

(provide :state-machine-utils) ;; end of state-machine-utils.l
