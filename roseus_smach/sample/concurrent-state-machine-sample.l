#!/usr/bin/env irteusgl
;; paralell-state-machine-sample.l
;; Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-mcahine-utils "package://roseus_smach/src/state-machine-utils.l")

(defclass washing-machine
 :super propertied-object
 :slots (door soap cloth))

(defmethod washing-machine
  (:init ()
   (setq door :close))
  (:open-door ()
   (warning-message 2 ";; open door~%")
   (setq door :open)
   t)
  (:put-cloth (clothes)
   (warning-message 2 ";; put ~A clothes~%" (length clothes))
   (setq cloth clothes)
   t)
  (:put-soap ()
   (warning-message 2 ";; put soap~%")
   (setq soap t)
   t)
  (:hoge ()
   (warning-message 2 ";; hoge~%")
   t)
  (:hoo ()
   (warning-message 2 ";; hoo~%")
   t)
  (:fuga ()
   (warning-message 2 ";; fuga~%")
   t)
  (:close-door ()
   (warning-message 2 ";; close door~%")
   (setq door :close)
   t)
  (:press-button ()
   (warning-message 2 ";; press button~%")
   (cond
     ((eq door :open)
      (warning-message 1 "you forgot close door!!~%"))
     ((null soap)
      (warning-message 1 "you forgot to use soap!!"))
     ((null cloth)
      (warning-message 1 "no cloth in washing machine!!~%"))
     (t
      (warning-message 4 "To keep your cloth clean is to keep your mind clean.")
      (return-from :press-button t)))
   (error)))

(defun make-sample-concurrent-state-machine ()
  (setq *wash* (instance washing-machine :init))
  (setq *sm*
        (make-state-machine
         '((:open-door -> (:put-soap :put-cloth))
           (:put-soap -> :hoo)
           (:hoo -> :close-door)
           (:put-cloth ->  :hoge)
           (:hoge -> :fuga)
           (:fuga -> :close-door)
           (:close-door -> :press-button)
           (:press-button -> :success))
         '((:open-door #'(lambda (&rest args) (send *wash* :open-door)))
           (:put-cloth #'(lambda (&rest args) (send *wash* :put-cloth '(:towel :t-shirt))))
           (:put-soap  #'(lambda (&rest args) (send *wash* :put-soap)))
           (:hoge  #'(lambda (&rest args) (send *wash* :hoge)))
           (:hoo  #'(lambda (&rest args) (send *wash* :hoo)))
           (:fuga  #'(lambda (&rest args) (send *wash* :fuga)))
           (:press-button #'(lambda (&rest args) (send *wash* :press-button)))
           (:close-door #'(lambda (&rest args) (send *wash* :close-door))))
         '(:open-door)
         '(:success))))

(defun init ()
  (ros::roseus "sample_parallel_state_machine")
  (make-sample-parallel-state-machine)
  (ros::ros-info "created state machine ~A" *sm*))

(warning-message 3 ";; (init)~%")

(defun demo ()
  (when (or (not (boundp '*sm*)) (not (boundp '*wash*)))
    (init))
  (exec-smach-with-spin *sm*))

(warning-message 3 ";; (demo)~%")

(provide :paralell-state-machine-sample)
