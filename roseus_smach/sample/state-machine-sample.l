#!/usr/bin/env roseus
;;
;; 3 type samples of State machine from SMACH tutorials
;;
(load "package://roseus_smach/src/state-machine.l")


;;
;; sample 1: simple state machine
;;  Euslisp version of http://wiki.ros.org/smach/Tutorials/Getting%20Started#Example
;;      https://raw.githubusercontent.com/rhaschke/executive_smach_tutorials/indigo-devel/examples/state_machine_simple.py
;;
(setq count 0)
(defun func-foo (&rest args)
  (format t "Execute state FOO~%")
  (cond ((< count 3) (incf count) :outcome1)
	(t :outcome2)))
(defun func-bar (&rest args)
  (format t "Execute state BAR~%")
  :outcome2)

(defun smach-simple ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :FOO 'func-foo))
    (send sm :add-node (instance state :init :BAR 'func-bar))
    ;; goal-states are generated in this method
    (send sm :goal-state (list :outcome4 :outcome5))

    ;; select a node as start-node
    (send sm :start-state :FOO)
    ;; from and to nodes are selected by name or symbol
    (send sm :add-transition :FOO :BAR :outcome1)
    (send sm :add-transition :FOO :outcome4 :outcome2)
    (send sm :add-transition :BAR :FOO :outcome2)
    sm ))

(defun smach-simple2 ()
  (let (sm)
    (setq sm
          (make-state-machine
           ;; define graph,  list of (<from-node> <transition> <to-node>)
           ;; if <transition> is ->, it corresponds when node returns t and !-> for nil.
           '((:foo :outcome2 :outcome4)
             (:foo :outcome1 :bar)
             (:bar :outcome2 :foo))
           ;; define function map
           '((:foo 'func-foo)  ;; foo returns :outcome1 3 times and then returns :outcome2
             (:bar 'func-bar)) ;; bar always returns :outcome2
           ;; initial state
           '(:foo)
           ;; goal state
           '(:outcome4)))))

(defun smach-simple3 ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:foo -> :outcome4)
             (:foo !-> :bar)
             (:bar -> :foo))
           '((:foo '(lambda (&rest args) (cond ((< count 3) (incf count) nil) (t t))))  ;; foo returns nil 3 times and then returns t
             (:bar '(lambda (&rest args) t)))                                           ;; bar always returns t
           '(:foo)
           '(:outcome4)))))

;;
;; sample 2: nodes can contain other state machine
;;  Euslisp version of http://wiki.ros.org/smach/Tutorials/Create%20a%20hierarchical%20state%20machine
;;      https://raw.githubusercontent.com/rhaschke/executive_smach_tutorials/indigo-devel/examples/state_machine_nesting2.py
;;
(defun func-bas (&rest args)
  (format t "Execute state BAS~%")
  :outcome3)

(defun smach-nested ()
  (let ((sm-top (instance state-machine :init))
	(sm-sub (instance state-machine :init)))
    ;; state instance can include other state-machine like function
    (send sm-top :add-node (instance state :init "SUB" sm-sub))
    (send sm-top :add-node (instance state :init "BAS" 'func-bas))
    (send sm-top :goal-state :outcome5)
    (send sm-top :start-state "BAS")
    (send sm-top :add-transition "BAS" "SUB" :outcome3)
    (send sm-top :add-transition "SUB" :outcome5 :outcome4)
    ;; node instance can be args of :add-node, :start-state, :add-transition
    (let ((foo-node (instance state :init "FOO" 'func-foo))
	  (bar-node (instance state :init "BAR" 'func-bar)))
      (send sm-sub :add-node foo-node)
      (send sm-sub :add-node bar-node)
      (send sm-sub :goal-state :outcome4)
      (send sm-sub :start-state foo-node)
      (send sm-sub :add-transition foo-node bar-node :outcome1)
      (send sm-sub :add-transition foo-node :outcome4 :outcome2)
      (send sm-sub :add-transition bar-node foo-node :outcome2))

    sm-top ))


(defun smach-simple-nested ()
  (let (sm-top sm-sub)
    (setq sm-sub
          (make-state-machine
           '((:foo :outcome2 :outcome4) ;; transitions
             (:foo :outcome1 :bar)
             (:bar :outcome2 :foo))
           '((:foo 'func-foo) ;; function maps
             (:bar 'func-bar))
           '(:foo)      ;; initial
           '(:outcome4) ;; goal
           ))
    (setq sm-top
          (make-state-machine
           '((:bas :outcome3 :sub) ;; transitions
             (:sub :outcome4 :outcome5))
           '((:bas 'func-bas)  ;; functon maps
             (:sub sm-sub))   ;; set "nestaed state machine"
           '(:bas)      ;; initial
           '(:outcome5) ;; goal
           ))
    sm-top))

;;
;; sample 3: A State machine reperesents only transitions between conditions.
;;           There is no local variable in state machine.
;;  Euslisp version of http://wiki.ros.org/smach/Tutorials/User%20Data
;;      https://raw.githubusercontent.com/rhaschke/executive_smach_tutorials/indigo-devel/examples/user_data2.py
;;
(defun func-foo-data (args)
  (format t "Execute state FOO~%")
  (unless (cdr (assoc 'foo-count args))
    (warn "foo-count is not set. Setting 0~%")
    (set-alist 'foo-count 0 args))
  (cond ((< (cdr (assoc 'foo-count args)) 3)
	 (incf (cdr (assoc 'foo-count args))) :outcome1)
	(t :outcome2)))
(defun func-bar-data (args)
  (format t "Execute state BAR~%")
  (format t "bar-count = ~A~%" (cdr (assoc 'bar-count args)))
  :outcome2)

(defun smach-userdata ()
  (let ((sm (instance state-machine :init)))
    ;; declare use all userdata keyword
    (send sm :arg-keys 'count)
    ;; remap from variable names in function to names in userdata
    (send sm :add-node
	  (instance state :init :FOO 'func-foo-data
		    :remap-list '((foo-count . count))))
    (send sm :add-node (instance state :init :BAR 'func-bar-data))
    (send sm :goal-state (list :outcome4 :outcome5))
    ;; set remap list later
    (send (send sm :node :BAR) :remap-list '((bar-count . count)))

    (send sm :start-state :FOO)
    (send sm :add-transition :FOO :BAR :outcome1)
    (send sm :add-transition :FOO :outcome4 :outcome2)
    (send sm :add-transition :BAR :FOO :outcome2)
    sm ))

(warn ";;(smach-simple)~%;;(smach-nested)~%;;(smach-userdata)~%")
