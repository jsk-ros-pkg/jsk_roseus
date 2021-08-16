(unless (find-package "ROSEUS_BT")
  (make-package "ROSEUS_BT" :nicknames (list "ROSEUS-BT")))

(in-package "ROSEUS_BT")

(export '(define-action-callback define-condition-callback
          action-node condition-node spin-once spin
          set-output ok))

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
             (not (zerop (third fn))))
    (let ((name (get-fn-sym fn)))
      (ros::ros-warn
       (format nil "Possibly harmful lambda-closure in #'~A! Try using '~A instead."
               name
               name)))))


;; macros
(defmacro define-action-callback (name arg-list &rest body)
  (user::with-gensyms (server goal get-result)
    `(prog1
       (defun ,name (,server ,goal)
         (labels ((set-output (name val)
                    (let ((msg (send ,server :feedback
                                     (intern (string-upcase name) *keyword-package*) val)))
                      (send ,server :publish-feedback msg)))
                  (ok ()
                    (send ,server :spin-once)
                    (not (send ,server :is-preempt-requested)))
                  (,get-result ,arg-list
                    (block ,name
                      ,@body)))
           (let (,@(mapcar
                    #'(lambda (k) `(,k (send ,goal :goal
                                             ,(intern (symbol-pname k) *keyword-package*))))
                    arg-list))
             (send ,server :set-succeeded
                   (send ,server :result :success
                         (,get-result ,@arg-list))))))
       (setf (get ',name :function-type) :action-node-callback)
       (setf (get ',name :function-documentation) ,(format nil "~S" arg-list)))))


(defmacro define-condition-callback (name arg-list &rest body)
  (user::with-gensyms (request get-result)
    `(prog1
       (defun ,name (,request)
         (labels ((,get-result ,arg-list
                    (block ,name
                      ,@body)))
           (let ((response (send ,request :response))
                 ,@(mapcar
                    #'(lambda (k) `(,k (send ,request ,(intern (symbol-pname k) *keyword-package*))))
                    arg-list))
             (send response :success (,get-result ,@arg-list))
             response)))
       (setf (get ',name :function-type) :condition-node-callback)
       (setf (get ',name :function-documentation) ,(format nil "~S" arg-list)))))


;; classes
(defclass action-node :super ros::simple-action-server)
(defmethod action-node
  (:init (ns spec &key execute-cb preempt-cb accept-cb groupname)
      (unless (eq (get (get-fn-sym execute-cb) :function-type) :action-node-callback)
        (ros::ros-error 
         (concatenate-string 
          "Wrong function type detected! "
          "Make sure to define execution callbacks using the define-action-callback macro"))
        (error "action-node-callback function expected for execute-cb"))
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
      self))

(defclass condition-node :super propertied-object)
(defmethod condition-node
  (:init (ns spec &key execute-cb)
    (unless (eq (get (get-fn-sym execute-cb) :function-type) :condition-node-callback)
      (ros::ros-error 
       (concatenate-string 
        "Wrong function type detected! "
        "Make sure to define execution callbacks using the define-condition-callback macro"))
      (error "condition-node-callback function expected for execute-cb"))

    (ros::advertise-service ns spec execute-cb)
    (push self *condition-list*)
    (if *action-list*
        (ros::ros-warn "Actions and Conditions detected in the same node! Start two separate nodes when reacitivity is required."))
    self))


;; functions
(defun spin-once ()
  (ros::spin-once)
  (dolist (ac *action-list*)
    (send ac :worker)))

(defun spin ()
  (while t
    (spin-once)
    (ros::sleep)))

(defun set-output (name val)
  (error "not defined in global scope"))

(defun ok () t)
