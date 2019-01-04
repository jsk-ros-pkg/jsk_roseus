;; state-machine-utils.l

(defun exec-state-machine (sm &optional (mydata '(nil))
                           &key (spin t) (hz 1) (root-name "SM_ROOT") iterate)
  "Execute state machine

Args:
  sm (state-machine): state machine
  mydata (alist): userdata for smach
  :spin (bool, default: t): call (ros::spin-once) every time before executing state if T, call nothing otherwise.
  :hz (integer, default: 1): rate of execution loop
  :iterate (t or function): Enable asking [Y/N] on each action execution if given t, if function is given, then it is called before execution. If function returns nil, next action is not executed. Otherwise the action is executed

Returns:
  the last active state
"
  (let ((insp (instance state-machine-inspector :init sm :root-name root-name)))
    (unix::sleep 2)
    (send sm :reset-state)
    (send insp :publish-structure) ;; publish once and latch
    (apply #'send sm :arg-keys (union (send sm :arg-keys) (mapcar #'car mydata)))

    (while (ros::ok)
      (when spin
        (ros::spin-once)
        (if (and (boundp '*ri*) *ri*) (send *ri* :spin-once)))
      (send insp :publish-status mydata)
      (when (send sm :goal-reached)
        (return))
      (when iterate
        (cond
          ((functionp iterate)
           (unless (funcall iterate (send sm :active-state))
             (ros::ros-warn "set abort in iteration")
             (return))
          (iterate
           (unless (y-or-n-p (format nil "Execute ~A ? "
                                     (send (send sm :active-state) :name)))
             (ros::ros-warn "aborting...")
             (return))
          (t (error "value of key :iterate must be t or function"))))))
      (send sm :execute mydata :step -1)
      (unix::usleep (round (/ 1e6 hz))))
    (send sm :active-state)))

(defun smach-exec (sm)
  "Deprecated function"
  (ros::ros-warn "This function is deprecated and will be removed. Use exec-state-machine")
  (exec-state-machine sm))
(defun exec-smach-with-spin (sm &optional mydata &key (hz 1) iterate)
  "Deprecated function"
  (ros::ros-warn "This function is deprecated and will be removed. Use exec-state-machine")
  (exec-state-machine sm mydata :hz hz :iterate iterate))

(defun make-state-machine (graph-list func-map initial-state goal-states
                           &key (exec-result t) (exec-failure-result nil) (parallel-exec-result t))
  "This is utility function to make state machine
e.g. (make-state-machine '((:a -> :b)
                           (:b -> :c)
                           (:b !-> :b_f)
                           (:b :skip :b_s)
                           (:b_f -> :fail)
                           (:b_s -> :end)
                           (:c -> (:d1 :d2))
                           (:d1 -> :end)
                           (:d2 -> :end))
                         '((:a 'action-a)
                           (:b 'action-b)
                           (:b_f 'action-b-f)
                           (:b_s 'action-b-s)
                           (:c 'action-c)
                           (:d1 'action-d1)
                           (:d2 'action-d2))
                          '(:a)
                          '(:end :fail))
    This makes state machine like:
      :a -> :b -> :c -> :d1 -> :end
             |  t  \ -> :d2 /    ^
         nil  -> :b_f -> :fail   |
             |                   |
        :skip -> :b_s ------------
    Each actions are assumed to implement like below:
    ```
    (defun action-a (&rest args)
     (if (eq (do-action-a) :success)
       t nil))
    ```
    By default, if `action-a` returns value t, then state transits as defined above.
    You can specify return value as `exec-result` key.
    Also, you can speficy edges with symbol, number and string as `(:b :skip :b_s)`.
    If the action body include references to global variables, it must be given as 'action-a (and not #'action-a)
    https://github.com/jsk-ros-pkg/jsk_roseus/pull/589#issuecomment-446071574
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
          (error "illegal type of func-map element: ~A[~A]~%" (eval (get-alist node func-map)) node))))

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
              (to-nodes (flatten (list (caddr connection))))
              (test (cadr (memq :test connection))))
          (unless from-nodes (setq from-nodes '(nil)))
          (unless to-nodes (setq to-nodes '(nil)))
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
                     ((or (numberp edge) (stringp edge) (symbolp edge)) edge)
                     (t (error "invalid connection ~A ~A ~A" from-node edge to-node)))
                    :test (if test test #'equal))))))
      sm)))

(provide :state-machine-utils) ;; end of state-machine-utils.l
