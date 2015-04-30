;; test-async-join-state-machine-actionlib.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "roseus_smach")
(ros::roseus "test_async_join_state_machine_actionlib")
(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")

(defun make-random-int-func (&optional userdata)
  (unix:sleep 3)
  (let ((randint 10))
    (warn "generated random ~A~%" randint)
    (setq *randint* randint)
    (set-alist :value randint userdata))
  t)

(defun add-two-ints-func (&optional userdata)
  (unix:sleep 3)
  (let ((val1 (send (elt (cdr (assoc :values userdata)) 0) :value))
        (val2 (send (elt (cdr (assoc :values userdata)) 1) :value)))
    (warn "~A + ~A~%" val1 val2)
    (setq *answer* (+ val1 val2)))
  t)

(defun int->multiply10actiongoal (val)
  (let ((goal (instance roseus_smach::Multiply10ActionGoal :init)))
    (send goal :goal :value val)
    goal))

(defun int->sub5actiongoal (val)
  (let ((goal (instance roseus_smach::Sub5ActionGoal :init)))
    (send goal :goal :value val)
    goal))

(defun make-async-join-state-machine ()
  (let ((sm (instance state-machine :init))
        (mul10-client (instance ros::simple-action-client :init
                                "multiply10" roseus_smach::Multiply10Action))
        (sub5-client (instance ros::simple-action-client :init
                               "sub5" roseus_smach::Sub5Action))
        random-int-value
        async-clients)

    (send sm :add-node
          (instance state :init :make-random-int
                    'make-random-int-func
                    :remap-list '((:value . random-int-value))))
    (send sm :add-node
          (actionlib-client-state :multiply-10
           mul10-client :async t :timeout 20 :retry t :key 'int->multiply10actiongoal))
    (send (send sm :node :multiply-10) :remap-list '((:goal . random-int-value)
                                                     (:async . async-clients)))
    (send sm :add-node
          (actionlib-client-state :sub-5
           sub5-client :async t :timeout 20 :retry t :key 'int->sub5actiongoal))
    (send (send sm :node :sub-5) :remap-list '((:goal . random-int-value)
                                               (:async . async-clients)))
    (send sm :add-node
          (instance async-join-state :init
                    :join :timeout 30
                    :remap-list '((:async . async-clients))))
    (send sm :add-node
          (instance state :init :sum 'add-two-ints-func
                    :remap-list '((:values . :results))))

    (send sm :arg-keys :results 'random-int-value 'async-clients :cancel '*answer*)
    (send sm :start-state :make-random-int)
    (send sm :goal-state (list :success))
    (send sm :add-transition :make-random-int :multiply-10 t)
    (send sm :add-transition :make-random-int :sub-5 t)
    (send sm :add-transition :multiply-10 :join t)
    (send sm :add-transition :sub-5 :join t)
    (send sm :add-transition :join :sum t)
    (send sm :add-transition :sum :success t)
    sm))
