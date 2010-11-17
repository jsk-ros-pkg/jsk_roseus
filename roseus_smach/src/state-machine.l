(load "euslib/jsk/graph.l")

;; manipulate a list of cons as a associative array
(defun set-alist (k v alist &key (key 'car) (test 'eq))
  (let ((cur-cons (assoc k alist :key key :test test)))
    (if cur-cons
	(progn (setf (cdr cur-cons) v) alist)
      (nconc alist (list (cons k v))))))

;; State machine is a direction graph.
;; Nodes defines state, and have action.
;; Goal states do nothing and returns its name.
(defclass state-machine
  :super graph
  :slots (active-state arg-keys))
(defmethod state-machine
  (:init () self)
  (:active-state
   (&rest args)
   (let ((node (car args)))
     (cond ((not node) active-state)
	   ((memq node nodes)
	    (if (send node :submachine)
		(send (send node :submachine) :reset-state))
	    (setq active-state node))
	   (t (setq active-state (find-if
		 #'(lambda(n)(send (send n :submachine) :active-state node))
		 (send self :sub-sm-node)))))
     ))
  (:reset-state () (send self :active-state start-state))
  (:start-state
   (&optional ss)
   (cond ((null ss) start-state)
	 ((derivedp ss state) (setq start-state ss))
	 (t (setq start-state (send self :node ss)))))
  ;; goal-state is a list of goal states
  (:goal-state
   (&optional gs)
   (if (null gs) (return-from :goal-state goal-state))
   (if (atom gs) (setq gs (list gs)))
   (setq gs (mapcar #'(lambda(g) (instance state :init g nil)) gs))
   (dolist (n gs) (send self :add-node n))
   (send-super :goal-state gs))
  (:goal-test (gs) (not (null (member gs goal-state))))
  (:goal-reached () (send self :goal-test active-state))
  ;;
  (:sub-sm-node () (remove-if-not #'(lambda(n)(send n :submachine)) nodes))
  ;;
  (:add-transition
   (from to exec-result)
   (if (not (derivedp from state))(setq from (or (send self :node from) from)))
   (if (not (derivedp to state))(setq to (or (send self :node to) to)))
   (send from :add-transition (cons exec-result to))
   (send-super :add-arc from to))
  ;; arg-keys is a list of userdata keys used by this state-machine
  (:arg-keys (&rest names)
   (if names (setq arg-keys names) arg-keys))

  (:execute-impl
   (userdata &key (step 0))
   ;; check if goal reached and returns goal state name
   (if (send self :goal-reached)
       (return-from :execute-impl (send active-state :name)))
   ;; execute in sub machine
   (if (and (not (eq 0 step)) (send active-state :submachine)
	    (not (send (send active-state :submachine) :goal-reached)))
       (return-from :execute-impl
	 (send active-state :execute userdata :step (1- step))))
   ;; execute once on this machine
   (let ((last-state active-state)
	 (exec-result (send active-state :execute userdata)))
     (when (not (assoc exec-result (send last-state :transitions)))
       (error "undefined transition ~A from ~A~%" exec-result last-state))
     (setq active-state (cdr (assoc exec-result (send last-state :transitions))))
     (if (send active-state :submachine)
	 (send (send active-state :submachine) :reset-state))
     exec-result))

  ;; set :step to execute once at selected depth.
  ;; 0->this 1->sub 2->subsub ... -1->deepest
  (:execute
   (userdata &key (step nil))
   (let (result
	 (args (mapcar #'(lambda(k)(cons k (cdr (assoc k userdata)))) arg-keys)))
     (if (null active-state) (send self :reset-state))
     (prog1
	 (cond ((null step)
		(while (not (send self :goal-reached))
		  (send self :execute-impl args))
		(send (send self :active-state) :name)) ;; reached state
	       (t (send self :execute-impl args :step step)))
       (dolist (item args) (set-alist (car item) (cdr item) userdata))
       )))
  )

;; action is function or state-machine instance
(defclass state
  :super node
  :slots (action transitions remap-list))
(defmethod state
  (:init
   (name act &key ((:remap-list rlst) nil))
   (setq action act remap-list rlst)
   (send-super :init name))
  (:transitions
   (&rest args)
   (if (car args) (setq transitions (car args))) transitions)
  (:add-transition (trans) (push trans transitions))
  ;; check if this state is state-machine, then return it
  (:submachine () (if (derivedp action state-machine) action nil))
  ;; remap userdata names
  (:remap-list
   (&rest args)
   (if (car args) (setq remap-list (car args))) remap-list)
  (:remap (userdata &key (invert nil))
	  (dolist (ft remap-list)
	    (when (and (not invert) (not (assoc (cdr ft) userdata)))
	      (nconc (list (cons (cdr ft) nil)) userdata))
	    (if invert
		(setf (car (assoc (car ft) userdata)) (cdr ft))
	      (setf (car (assoc (cdr ft) userdata)) (car ft)))))
  ;;
  (:execute
   (userdata &key (step nil))
   (let (res)
     (send self :remap userdata :invert nil) ;; remap
     (setq res (cond ((send self :submachine)
		      (send action :execute userdata :step step))
		     ((functionp action) (funcall action userdata))
		     (t (send self :name))))
     (send self :remap userdata :invert t) ;; unremap
     res ))
  )
