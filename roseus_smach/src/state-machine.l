;; state-machine.l

;; manipulate a list of cons as a associative array
(defun set-alist (k v alist &key (key 'car) (test 'eq))
  (let ((cur-cons (assoc k alist :key key :test test)))
    (if cur-cons
        (progn (setf (cdr cur-cons) v) alist)
        (nconc alist (list (cons k v))))))

;; State machine is a direction graph.
;; Nodes defines state, and have action.
;; Goal states do nothing and returns its name.
;; active-state, start-state may have multiple states and can execute simultaneously.
;; [TODO] currently executing submachine parallel is not yet supported (2015/4/7)
(defclass state-machine
  :super graph
  :slots (active-state arg-keys
          parallel-action-p parallel-exec-result parallel-trans-test))
(defmethod state-machine
  (:init (&key (parallel nil))
     (setq parallel-action-p parallel)
     self)
  (:active-state
   (&rest args)
   (let ((node (car args)))
     (cond ((null node) ;; getter
            (case (length active-state)
              (1 (car active-state))
              (t active-state)))
           ((listp node) ;; setter for multiple nodes
            (unless (equal node (intersection node nodes))
              (warning-message 2 "some nodes of [~A] are not included in [~A]~%" node nodes)
              (error))
            (setq active-state node))
           ((memq node nodes) ;; setter for single node
            (if (send node :submachine)
                (send (send node :submachine) :reset-state))
            (setq active-state (list node)))
           (t (setq active-state (find-if
                                  #'(lambda(n)(send (send n :submachine) :active-state node))
                                  (send self :sub-sm-node)))))
     ))
  (:next-arc-list
   (&optional state)
   (send state :arc-list))
  (:parallel-exec-result
   (&optional res)
   (cond ((null res) parallel-exec-result)
         (t (setq parallel-exec-result res))))
  (:reset-state () (send self :active-state start-state))
  (:start-state
   (&optional ss)
   (cond ((null ss) ;; getter
          (case (length start-state)
          (1 (car start-state))
          (t start-state)))
         ((listp ss) ;; setter for multiple nodes
            (unless (equal ss (intersection ss (send-all nodes :name)))
              (warning-message 2 "some nodes of [~A] are not included in [~A].~%" ss (send-all nodes :name))
              (error))
            (setq start-state (mapcar #'(lambda (n) (send self :node n)) ss)))
         ((derivedp ss state) (setq start-state (list ss))) ;; setter for single node
         (t (setq start-state (list (send self :node ss))))))
  ;; goal-state is a list of goal states
  (:goal-state
   (&optional gs)
   (if (null gs) (return-from :goal-state goal-state))
   (if (atom gs) (setq gs (list gs)))
   (setq gs (mapcar #'(lambda(g) (instance state :init g nil)) gs))
   (dolist (n gs) (send self :add-node n))
   (send-super :goal-state gs))
  (:goal-test (gs) (not (null (intersection (flatten (list gs)) goal-state))))
  (:goal-reached () (send self :goal-test active-state))
  ;;
  (:sub-sm-node () (remove-if-not #'(lambda(n)(send n :submachine)) nodes))
  ;; @ from,to: state instance or its name
  ;; @ exec-result: trans state when execution at from-state returns this.
  ;;                must be {number,string,symbol}.
  (:add-transition
   (from to exec-result &key (test #'equal))
   (when (not (derivedp from state))
     (if (send self :node from)
      (setq from (send self :node from))
      (ros::ros-warn "add-transition: state ~A is not yet added. created new node with this name." from)))
   (when (not (derivedp to state))
     (if (send self :node to)
      (setq to (send self :node to))
      (ros::ros-warn "add-transition: state ~A is not yet added. created new node with this name." to)))
   (if (and from to)
       (send self :add-arc from to exec-result test)))
  (:add-arc (from to val test)
            (send self :add-arc-from-to from to val test))
  (:add-arc-from-to (from to val test)
                    (instance transition :init from to val test))
  ;; arg-keys is a list of userdata keys used by this state-machine
  (:arg-keys (&rest names)
             (if names (setq arg-keys names) arg-keys))

  (:execute-impl
   (userdata &key (step 0))
   ;; check if goal reached and returns goal state name
   (when (send self :goal-reached)
       (return-from :execute-impl
         (case (length active-state)
           (1 (send (car active-state) :name))
           (t (send-all active-state :name)))))
   ;; execute in sub machine
   (let ((active-state1 (car active-state)))
     (when (and (not (eq 0 step))
                (send active-state1 :submachine)
                (not (send (send active-state1 :submachine) :goal-reached)))
       (return-from :execute-impl
         (list (send active-state1 :execute userdata :step (1- step))))))
#|
   (when (and (not (eq 0 step))
            (derivedp active-state state) ;; single node
            (send active-state :submachine)
            (not (send (send active-state :submachine) :goal-reached)))
       (return-from :execute-impl
         (send active-state :execute userdata :step (1- step))))
|#
   ;; execute once on this machine
   (let (ret next-active-state)
     (dolist (astate active-state)
       (let* ((last-state astate)
              (trans (send self :next-arc-list astate))
              (exec-result  (send last-state :execute userdata)))
         (ros::ros-debug "trans: ~A" trans)
         (setq trans (remove-if-not #'(lambda(tr)(send tr :check exec-result)) trans))
         (case (length trans)
           (0 (error "undefined transition ~A from ~A~%" exec-result last-state))
           (1 t) ;; OK
           (t
            (case (length active-state)
                (1 (warn "multiple transitions ~A from ~A~%" exec-result last-state))
                (t t))))
         ;; check if active state has changed
         (when (not (eq astate last-state))
           (error "active state has not changed ~A -> ~A~%" last-state astate)
           (return-from :execute-impl (send astate :name)))
         (setq next-active-state (append next-active-state (send-all trans :to)))
         (if (send astate :submachine)
             (send (send astate :submachine) :reset-state))
         (push exec-result ret)))
     (setq active-state (unique next-active-state))
     ret))

  ;; set :step to execute once at selected depth.
  ;; 0->this 1->sub 2->subsub ... -1->deepest
  (:execute
   (userdata &key (step nil))
   (let ((args (if arg-keys (mapcar #'(lambda (k) (cons k (cdr (assoc k userdata)))) arg-keys) userdata))
         result)
     (when (null active-state)
       (send self :reset-state))
     (setq result
           (prog1
               (cond ((null step)
                      (while (not (send self :goal-reached))
                             (send self :execute-impl args))
                      (send-all active-state :name)) ;; reached state
                     (t (send self :execute-impl args :step step)))
             (dolist (item args) (set-alist (car item) (cdr item) userdata))
             )
           )
     (case (length result)
       (1 (car result))
       (t result))))
  )

;; action is function or state-machine instance
(defclass state
  :super node
  :slots (action remap-list))
(defmethod state
  (:init
   (name act &key ((:remap-list rlst) nil))
   (setq action act remap-list rlst)
   (send-super :init name))
  ;; check if this state is state-machine, then return it
  (:submachine () (if (derivedp action state-machine) action nil))
  ;; remap userdata names
  (:remap-list
   (&rest args)
   (if (car args) (setq remap-list (car args))) remap-list)
  (:remap (userdata &key (invert nil))
   (dolist (ft remap-list)
     (when (and (not invert) (not (assoc (cdr ft) userdata)))
       (nconc userdata (list (cons (cdr ft) nil))))
     (if invert
         (setf (car (assoc (car ft) userdata)) (cdr ft))
         (setf (car (assoc (cdr ft) userdata)) (car ft)))))
  ;;
  (:execute
   (userdata &key (step nil))
   (let (res)
     (send self :remap userdata :invert nil) ;; remap
     (setq res
           (cond ((send self :submachine)
                  (send action :execute userdata :step step))
                 ((functionp action) (funcall action userdata))
                 (t (send self :name))))
     (send self :remap userdata :invert t) ;; unremap
     res ))
  )

(defclass transition
  :super costed-arc
  :slots (value testfunc))
(defmethod transition
  (:init
   (from to val test)
   (send self :name val)
   (setq value val testfunc test)
   (send-super :init from to 1))
  (:check
   (arg)
   (funcall testfunc arg value))
  )

(provide :state-machine)
