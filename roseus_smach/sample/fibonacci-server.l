#!/usr/bin/env roseus
;;;
;;; steta-machine version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_server.py
;;;
(load "package://roseus_smach/src/state-machine-actionlib.l")
(ros::roseus-add-msgs "actionlib_tutorials")


(defun func-fibonacci (userdata)
  (let* ((goal (cdr (assoc :goal userdata)))
         (result (cdr (assoc :result userdata)))
         (feedback (cdr (assoc :feedback userdata)))
         (cancel (cdr (assoc :cancel userdata)))
         (seq (send result :sequence)) (next 0))
    (when (send cancel :enabled-p) (return-from func-fibonacci))
    (when (/= (length seq) (send goal :order))
      (setq seq (make-array (send goal :order) :element-type :integer))
      (send result :sequence seq)
      (setf (elt seq 0) 1)
      (setf (elt seq 1) 1))
    (send feedback :sequence seq)
    (while t
      (cond ((eq next (send goal :order))
             (return-from func-fibonacci t))
            ((zerop (elt seq next))
             (setf (elt seq next) (+ (elt seq (- next 1))
                                     (elt seq (- next 2))))
             (return-from func-fibonacci nil))
            (t (incf next))))
    ))
(defun func-sleep (&rest args) (unix::sleep 1) t)

(defun fibonacci-smach ()
  (let ((sm (instance state-machine :init)))
    ;; remap from variable names in function to names in userdata
    (send sm :add-node (instance state :init :CALC 'func-fibonacci))
    (send sm :add-node (instance state :init :SLEEP1 'func-sleep))
    (send sm :add-node (instance state :init :SLEEP2 'func-sleep))
    (send sm :goal-state (list :success))
    (send sm :arg-keys :goal :feedback :result :cancel)

    (send sm :start-state :CALC)
    (send sm :add-transition :CALC :SLEEP1 nil)
    (send sm :add-transition :CALC :success t)
    (send sm :add-transition :SLEEP1 :SLEEP2 t)
    (send sm :add-transition :SLEEP2 :CALC t)
    sm ))

(defun run-fibonacci-server ()
  (ros::roseus "fibonacci")
  (warn "fibonacci server is working and press any key to stop.")
  (setq sm (fibonacci-smach))
  (setq s (instance state-machine-action-server :init "/fibonacci"
                    actionlib_tutorials::fibonacciaction sm))
  (ros::rate 10)
  (do-until-key
      (ros::spin-once)
    (send s :worker)
    (ros::sleep)))

(when (substringp "__log:=" (car (last lisp::*eustop-argument*)))
  (run-fibonacci-server))
