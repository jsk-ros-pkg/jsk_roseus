(unless (find-package "ROSEUS_BT")
  (make-package "ROSEUS_BT" :nicknames (list "ROSEUS-BT")))

(in-package "ROSEUS_BT")

(export '(action-node condition-node spin-once spin))

(defvar *action-list*)
(defvar *condition-list*)


;; utility
(defun get-fn-sym (fn)
  (when (functionp fn)
    (cond
      ((symbolp fn) fn)
      ((and (listp fn) (memq (car fn) '(lambda lambda-closure)))
       (cadr fn))
      ((derivedp fn compiled-code)
       (send fn :name))
      (t nil))))

(defun check-fn-closure (fn)
  (when (and (consp fn)
             (eql (car fn) 'lambda-closure)
             (or (not (zerop (third fn)))
                 (not (zerop (fourth fn)))))
    (let ((name (get-fn-sym fn)))
      (if name
          (ros::ros-warn
           (format nil "Possibly harmful lambda-closure in #'~A! Try using '~A instead."
                   name
                   name))
          (ros::ros-warn
           (format nil "Possibly harmful lambda context in ~A! Reseting to 0..."
                   (append (subseq fn 0 5) '(...)))
           (setf (third fn) 0)
           (setf (fourth fn) 0))))))


;; classes
(defclass action-node :super ros::simple-action-server)
(defmethod action-node
  (:init (ns spec &key execute-cb preempt-cb accept-cb groupname)
      (check-fn-closure execute-cb)
      (check-fn-closure preempt-cb)
      (check-fn-closure accept-cb)

      (send-super :init ns spec
              :execute-cb execute-cb
              :preempt-cb preempt-cb
              :accept-cb accept-cb
              :groupname groupname)
      (push self *action-list*)
      (if *condition-list*
          (ros::ros-warn "Actions and Conditions detected in the same node! Start two separate nodes when reacitivity is required."))
      self)
  (:set-output (name val)
    (let ((msg (send self :feedback (intern (string-upcase name) *keyword-package*) val)))
      (send self :publish-feedback msg)))
  (:ok ()
     (send self :spin-once)
     (not (send self :is-preempt-requested))))

(defclass condition-node :super propertied-object :slots (execute-cb))
(defmethod condition-node
  (:init (ns spec &key ((:execute-cb exec-cb)) groupname)
    (check-fn-closure exec-cb)
    (setq execute-cb exec-cb)
    (if groupname
        (ros::advertise-service ns spec #'send self :request-cb :groupname groupname)
        (ros::advertise-service ns spec #'send self :request-cb))
    (push self *condition-list*)
    (if *action-list*
        (ros::ros-warn "Actions and Conditions detected in the same node! Start two separate nodes when reacitivity is required."))
    self)
  (:request-cb (msg)
    (let ((response (send msg :response)))
      (send response :success (funcall execute-cb self msg))
      response)))


;; functions
(defun spin-once ()
  (ros::spin-once)
  (dolist (ac *action-list*)
    (send ac :worker)))

(defun spin ()
  (while t
    (spin-once)
    (ros::sleep)))
