#!/usr/bin/env irteusgl
;; parallel-state-machine-sample.l
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

(defun make-sample-parallel-state-machine ()
  (setq *wash* (instance washing-machine :init))
  (setq *sm*
        (make-state-machine
         '((:open-door -> (:put-cloth :put-soap))
           (:put-cloth ->  :close-door)
           (:put-soap -> :close-door)
           (:close-door -> :press-button)
           (:press-button -> :success))
         '((:open-door '(lambda (&rest args) (send *wash* :open-door)))
           (:put-cloth '(lambda (&rest args) (send *wash* :put-cloth '(:towel :t-shirt))))
           (:put-soap  '(lambda (&rest args) (send *wash* :put-soap)))
           (:press-button '(lambda (&rest args) (send *wash* :press-button)))
           (:close-door '(lambda (&rest args) (send *wash* :close-door))))
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
  (exec-state-machine *sm*))

(warning-message 3 ";; (demo)~%")

(provide :parallel-state-machine-sample)
