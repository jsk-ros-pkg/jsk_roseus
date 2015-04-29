;; test-async-join-state-machine-actionlib.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")

(defun make-random-int-func (&rest userdata)
  (unix:sleep 3)
  (set-alist 'random-int-value (random 100) userdata)
  t)

(defun add-two-ints-func (&rest userdata)
  (unix:sleep 3)
  (apply #'+ (cdr (assoc :values userdata)))
  t)

(defun int->multiply10actiongoal (val)
  (instance roseus_smach::Multiply10ActionGoal :init
                  :goal :value val))

(defun int->sub5actiongoal (val)
  (instance roseus_smach::Sub5ActionGoal :init
            :goal :value val))


(defun make-async-join-state-machine ()
  (let ((sm (instance state-machine :init))
        (mul10-client (instance ros::simple-action-client :init
                                "multiply10" roseus_smach::Multiply10Action))
        (sub5-client (instance ros::simple-action-client :init
                               "sub5" roseus_smach::Sub5Action))
        (average-client (instance ros::simple-action-client :init
                                  "average" roseus_smach::AverageTwoIntAction))
        random-int-value)
    (send sm :add-node
          (instance state :init :make-random-int
                    'make-random-int-func))
    (send sm :add-node
          (actionlib-client-state :multiply-10
           mul10-client :timeout 20 :retry t :key 'int->multiply10actiongoal))
    (send (send sm :node :multiply-10) :remap-list '((:goal . random-int-value)))
    (send sm :add-node
          (actionlib-client-state :sub-5
           sub5-client :timeout 20 :retry t :key 'int->sub5actiongoal))
    (send (send sm :node :sub-5) :remap-list '((:goal . random-int-value)))
    (send sm :add-node
          (instance async-join-state :init
                    :join :timeout 30))
    (send sm :add-node
          (instance state :init :sum 'sum-two-int
                    :remap-list '((:values . :results))))

    (send sm :arg-keys :results 'random-int-value :cancel)
    (send :start-state :make-random-int)
    (send :goal-state (list :success))
    (send sm :add-transition :make-random-int :multiply-10 t)
    (send sm :add-transition :make-random-int :sub-5 t)
    (send sm :add-transition :multiply-10 :join t)
    (send sm :add-transition :sub-5 :join t)
    (send sm :add-transition :join :sum t)
    (send sm :add-transition :sum :success t)
    sm))




(provide :test-async-join-state-machine-actionlib) ;; end of test-async-join-state-machine-actionlib.l
