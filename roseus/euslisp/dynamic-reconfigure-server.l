(ros::load-ros-manifest "dynamic_reconfigure")
;;
;; refer to dynamic_reconfigure/src/dynamic_reconfigure/encoding.py
;;
(defun encode-description (descr) ;; description -> msg
  (let ((msg (instance dynamic_reconfigure::ConfigDescription :init)))
    (send msg :max (encode-config (get descr :max)))
    (send msg :min (encode-config (get descr :min)))
    (send msg :dflt (encode-config (get descr :defaults)))
    (let (lst)
      (dolist  (param (get descr :config_description))
        (let ((pmd (instance dynamic_reconfigure::ParamDescription :init
                             :name (cdr (assoc :name param))
                             :type (cdr (assoc :type param))
                             :level (cdr (assoc :level param))
                             :description (cdr (assoc :description param))
                             :edit_method (cdr (assoc :edit_method param)))))
          (push pmd lst)))
      ;; TODO: default group is only supported
      (send msg :groups 
            (list (instance dynamic_reconfigure::Group :init
                            :name "Default"
                            :parameters (nreverse lst)))))
    msg))


(defun encode-config (config)
  (let ((msg (instance dynamic_reconfigure::Config :init)))
    (dolist (cf config)
      (let ((k (car cf))
            (v (cdr cf)))
       (cond
        ((integerp v)
         (send msg :ints
               (append (send msg :ints) (list (instance dynamic_reconfigure::IntParameter :init
                                                        :name k :value v)))))
        ((or (eq v nil)
             (eq v t))
         (send msg :bools
               (append (send msg :bools) (list (instance dynamic_reconfigure::BoolParameter :init
                                                         :name k :value v)))))
        ((stringp v)
         (send msg :strs
               (append (send msg :strs) (list (instance dynamic_reconfigure::StrParameter :init
                                                        :name k :value v)))))
        ((floatp v)
         (send msg :doubles
               (append (send msg :doubles) (list (instance dynamic_reconfigure::DoubleParameter :init
                                                        :name k :value v)))))
        )))
    msg
    ))
(defun decode-description (msg) ;; msg -> description
  (let (descr
        (mins (decode-config (send msg :min)))
        (maxes (decode-config (send msg :max)))
        (defaults (decode-config (send msg :dflt))))
    (dolist (pm (send msg :parameters))
      (let ((nm (send pm :name)))
        (push
         (list
          (cons :name nm)
          (cons :min (cdr (assoc nm mins :test #'equal)))
          (cons :max (cdr (assoc nm maxes :test #'equal)))
          (cons :default (cdr (assoc nm defaults :test #'equal)))
          (cons :type (send pm :type))
          (cons :description (send pm :description))
          (cons :edit_method (send pm :edit_method)))
         descr)
        ))
    (nreverse dscr)
    ))
(defun decode-config (msg)
  (let ((lst
         (append (send msg :bools)
                 (send msg :ints)
                 (send msg :strs)
                 (send msg :doubles)))
        ret)
    (dolist (l lst)
      (push
       (cons (send l :name)
             (send l :value)) ret))
    (nreverse ret)))

;;
;; refer to dynamic_reconfigure/src/dynamic_reconfigure/server.py
;;
(defclass reconfigure_server
  :super propertied-object
  :slots (type
          config
          description
          callbackfunc ;; (defun func (cfg level) (setting-parameter cfg) cfg)
          param-desc-topic
          param-updates-topic
          service-name
          )
  )

(defmethod reconfigure_server
  (:init
   (init_type cbfunc)
   (setq type init_type)
   (setq callbackfunc cbfunc)
   (setq config (get init_type :defaults))
   (setq description (encode-description init_type))
   (send self :copy-from-parameter-server)
   (send self :clamp config)

   (let ((nname (ros::get-name)))
     (setq param-desc-topic (format nil "~A/parameter_descriptions" nname))
     (ros::advertise param-desc-topic
                     dynamic_reconfigure::ConfigDescription 1 t)
     (setq param-updates-topic (format nil "~A/parameter_updates" nname))
     (ros::advertise param-updates-topic
                     dynamic_reconfigure::Config 1 t)
     (ros::publish param-desc-topic description)
     (send self :change-config config (get init_type :all_level))
     (setq service-name (format nil "~A/set_parameters" nname))
     (ros::advertise-service service-name dynamic_reconfigure::Reconfigure
                             #'send self :set-callback))
   )
  (:update-configuration
   (changes)
   (let ((new-config (copy-object config)))
     (dolist (cf changes)
       (let ((elm
              (assoc (car cf) new-config :test #'equal)))
         (setf (car elm) (car cf))
         (setf (cdr elm) (cdr cf))))
     (send self :clamp new-config)
     (send self :change-config new-config
           (send self :calc-level new-config config))
     ))
  (:change-config
   (cfg level)
   (setq config (funcall callbackfunc cfg level))
   (unless config
     (warn ";; callbackfunc(~A) did not return config..." callbackfunc))
   (send self :copy-to-parameter-server)
   (let ((ret-cfg
          (encode-config config)))
     (ros::publish param-updates-topic ret-cfg)
     ret-cfg))
  (:copy-from-parameter-server
   ()
   ;; not implemented yet
   )
  (:copy-to-parameter-server
   ()
   ;; not implemented yet
   )
  (:calc-level
   (cfg1 cfg2)
   (let ((lv 0)
         (desclst (get type :config_description)))
     (dolist (l desclst)
       (when (not (equal (cdr (assoc (cdr (assoc :name l)) cfg1 :test #'equal))
                         (cdr (assoc (cdr (assoc :name l)) cfg2 :test #'equal))))
         (setq lv (logior lv (cdr (assoc :level l))))
         ))
     lv))
  (:clamp
   (cfg) ;; destructive
   (let ((maxlst (get type :max))
         (minlst (get type :min))
         (desclst (get type :config_description)))
     (dolist (l desclst)
       (let ((maxval (cdr (assoc (cdr (assoc :name l)) maxlst :test #'equal)))
             (minval (cdr (assoc (cdr (assoc :name l)) minlst :test #'equal)))
             (val (cdr (assoc (cdr (assoc :name l)) cfg :test #'equal))))
         (if (and (not (equal maxval ""))
                  (and (numberp val)
                       (numberp maxval)
                       (> val maxval)))
             (setf (cdr (assoc (cdr (assoc :name l)) cfg :test #'equal)) maxval)
           (if (and (not (equal minval ""))
                    (and (numberp val)
                         (numberp maxval)
                         (< val minval)))
               (setf (cdr (assoc (cdr (assoc :name l)) cfg :test #'equal)) minval)
             ))))
     ))
  (:set-callback
   (req)
   (let ((res (send req :response)))
     (send res :config
           (send self :update-configuration (decode-config (send req :config))))
     res))
  )

;;
;; dynamic_reconfigure setting for roseus
;;
;; TODO: cfg file load is not supported
(defmacro def-dynamic-reconfigure-server (descriptions callback)
  `(instance
    reconfigure_server :init
    (make-description-obj
     (list ,@(mapcar #'(lambda (x) `(make-parameter-desc ,@x)) descriptions)))
    ,callback))
(defmacro make-parameter-desc (&rest args)
  (let* ((tp (elt args 1))
         (tpstr (cond
                 ((eq 'int_t tp) "int")
                 ((eq 'double_t tp) "double")
                 ((eq 'str_t tp) "str")
                 ((eq 'bool_t tp) "bool")
                 (t  (warn ";; unknown type ~A" tp))))
         ret)
    (setq ret
          (list
           (cons :name (elt args 0))
           (cons :type tpstr)
           (cons :level (elt args 2))
           (cons :description (elt args 3))))
    (setq ret (append ret (list (cons :default
                                      (if (> (length args) 4)
                                          (cond
                                           ((eq 'int_t tp) (round (elt args 4)))
                                           ((eq 'double_t tp) (float (elt args 4)))
                                           ((eq 'str_t tp) (string (elt args 4)))
                                           ((eq 'bool_t tp) (if (elt args 4) t nil)))
                                        (cond
                                         ((eq 'int_t tp) 0)
                                         ((eq 'double_t tp) 0.0)
                                         ((eq 'str_t tp) "")
                                         ((eq 'bool_t tp) nil))
                                        )))))
    (setq ret (append ret (list (cons :min
                                      (if (> (length args) 5)
                                          (cond
                                           ((eq 'int_t tp) (round (elt args 5)))
                                           ((eq 'double_t tp) (float (elt args 5)))
                                           ((eq 'str_t tp) (string (elt args 5)))
                                           ((eq 'bool_t tp) (if (elt args 5) t nil)))
                                        (cond
                                         ((eq 'int_t tp) lisp::most-negative-fixnum)
                                         ((eq 'double_t tp) lisp::most-negative-float)
                                         ((eq 'str_t tp) "")
                                         ((eq 'bool_t tp) nil))
                                        )))))
    (setq ret (append ret (list (cons :max
                                      (if (> (length args) 6)
                                          (cond
                                           ((eq 'int_t tp) (round (elt args 6)))
                                           ((eq 'double_t tp) (float (elt args 6)))
                                           ((eq 'str_t tp) (string (elt args 6)))
                                           ((eq 'bool_t tp) (if (elt args 6) t nil)))
                                        (cond
                                         ((eq 'int_t tp) lisp::most-positive-fixnum)
                                         ((eq 'double_t tp) lisp::most-positive-float)
                                         ((eq 'str_t tp) "")
                                         ((eq 'bool_t tp) t))
                                        )))))
    (setq ret (append ret (list (cons :edit_method
                                      (if (> (length args) 7)
                                          (elt args 7) "")))))
    `',ret
    ))


(defun make-description-obj (lst)
  (let ((obj (instance propertied-object))
        max-lst
        min-lst
        level-lst
        type-lst
        default-lst
        (clevel 0))
    (setf (get obj :config_description) lst)
    (dolist (l lst)
      (let ((nm (cdr (assoc :name l))))
        (push (cons nm (cdr (assoc :max l))) max-lst)
        (push (cons nm (cdr (assoc :min l))) min-lst)
        (push (cons nm (cdr (assoc :level l))) level-lst)
        (push (cons nm (cdr (assoc :type l))) type-lst)
        (push (cons nm (cdr (assoc :default l))) default-lst)
        (setq clevel (logior clevel (cdr (assoc :level l))))
        ))
    (setf (get obj :max) (nreverse max-lst))
    (setf (get obj :min) (nreverse min-lst))
    (setf (get obj :level) (nreverse level-lst))
    (setf (get obj :type) (nreverse type-lst))
    (setf (get obj :defaults) (nreverse default-lst))
    (setf (get obj :all_level) clevel)
    obj))
#|
;; sample usage
(ros::roseus "reconfigure_server")
(setq *reconfigure-server*
      (def-dynamic-reconfigure-server
        ;;; ((name type level description (default) (min) (max) (edit_method)) ... )
        (("int_param"  int_t  1  "Int parameter" 0  -10  10)
         ("double_param"  double_t  2  "double parameter" 0.0  -2.0  10.0)
         ("str_param"  str_t  4  "String parameter" "foo")
         ("bool_param"  bool_t  8  "Boolean parameter"  nil))
        ;;; callbackfunc (defun func (config level) ) -> return updated-config
        #'(lambda (cfg level)
            (pprint cfg)
            ;; have to set parameter to user application
            cfg)))
(while (ros::ok)
  (ros::spin-once))
|#
