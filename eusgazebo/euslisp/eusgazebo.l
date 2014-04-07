;; eusgazebo-core.l

(load "package://eusgazebo/euslisp/eusgazebo-util.l")
(load "package://eusgazebo/euslisp/mass-property-util.l")

(ros::load-ros-manifest "gazebo_msgs")

(ros::roseus "eusgazebo_client")


(defclass eusgazebo
  :super propertied-object
  :slots (model-list ground)
  )

(defmethod eusgazebo
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; initialize
  (:init
   ()
   
   ;; start gzserver
   (warning-message 2 "launch gzserver~%")
   (unix:system "mkdir /tmp/gazebo_model -p")
   (unix:system "pkill gzserver")
   (unix:system "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/tmp/gazebo_model rosrun gazebo_ros gzserver &")
   ;; subscribe gazebo state
   (ros::subscribe "/gazebo/model_states"
		   gazebo_msgs::ModelStates #'send self :gazebo-model-state-cb)
   ;; publish eus state
   (ros::advertise "/gazebo/set_model_state" gazebo_msgs::ModelState 1)
   ;; make ane view ground on eus viewer
   (unless ground (setq ground (make-cube 5000 5000 1)))
   (cond ((and (boundp '*irtviewer*) (not (member ground (send *irtviewer* :objects))))
	  (objects ground))
	 (t
	  (objects (list ground))))
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; add eus model to gazebo world
  (:add-model
   (model
    &key (static nil) (urdf-dir "/tmp/gazebo_model"))

   ;; check if already added
   (when (member model model-list)
     (return-from :add-model nil))
   
   ;; check mass property
   (overwrite-mass-property-if-too-small model)

   ;; overwrite model name
   (cond
    ;; deal with the objects with no name (objects with no name cause an error in converting)
    ((null (send model :name)) (send model :name "no-name"))
    ;; deal with the objects with symbol name
    ((symbolp (send model :name)) (send model :name (string-downcase (send model :name)))))

   ;; set gazebo model name
   (send model :put :gazebo-model-name (send model :name))
   (let* ((obj-cnt 0))
     (while t
       (let* ((model-file-path
	       (format nil "~a/~a" urdf-dir (send model :get :gazebo-model-name))))
	 (unless (probe-file model-file-path) (return)))
       (send model :put :gazebo-model-name (format nil "~a-~a" (send model :name) obj-cnt))
       (incf obj-cnt)))

   ;; convert urdf
   (let* (urdf-model-path)
     (setq urdf-model-path (irteus2urdf-for-gazebo model :name (send model :get :gazebo-model-name) :urdf-dir urdf-dir :static static))
     (send model :put :urdf-model-path urdf-model-path))

   ;; spawn model
   (spawn-model-to-gazebo (send model :get :urdf-model-path) :model-name (send model :get :gazebo-model-name) :model-coords (send model :copy-worldcoords))

   ;; add to model-list
   (setq model-list (append model-list (list model)))
   (unix::usleep (* 100 1000))
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; delete model from gazebo world
  (:delete-model
   (model)

   ;; check if already added
   (unless (member model model-list)
     (return-from delete-model nil))
   
   ;; delete model
   (delete-model-to-gazebo (send model :get :gazebo-model-name))

   ;; delete to model-list
   (setq model-list (remove model model-list))
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; callback function of model_states topic
  (:gazebo-model-state-cb
   (msg)

   (dotimes (i (length (send msg :name)))
     (let* ((name (elt (send msg :name) i))
	    (pose (elt (send msg :pose) i))
	    (twist (elt (send msg :twist) i))
	    (model (find-if #'(lambda (_model) (string= (send _model :get :gazebo-model-name) name)) model-list)))
       (when model
	 (send model :put :gazebo-pose (pose->coords pose)))
       )))
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; update
  (:update
   ()
   (ros::spin-once)
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; apply eus state to gazebo
  (:eus2gzb
   (&optional model)

   (cond (model
	  (let* ((msg (instance gazebo_msgs::ModelState :init)))
	    (send msg :model_name (send model :get :gazebo-model-name))
	    (send msg :pose (coords->pose (send model :copy-worldcoords)))
	    (ros::publish "/gazebo/set_model_state" msg)))
	 (t ;; call for all drawed objects
	  (dolist (model model-list) (send self :eus2gzb model))))
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; apply gazebo state to eus
  (:gzb2eus
   (&optional model (update t))

   (when update
     (send self :update))
   (cond (model
	  (cond ((send model :get :gazebo-pose)
		 (send model :newcoords
		       (send model :get :gazebo-pose)))
		(t
		 (warning-message 1 "failed to get gazebo pose~%"))))
	 (t ;; call for all drawed objects
	  (dolist (model model-list) (send self :gzb2eus model nil))))
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; apply gazebo state to eus in loop
  (:gzb2eus-loop
   (&optional model)

   (setq *gzb-loop-flag* t)
   (do-until-key
    (unless *gzb-loop-flag*
      (return-from nil nil))
    (send self :gzb2eus model)
    (send *irtviewer* :draw-objects)
    (x::window-main-one)
    (unix::usleep (* 10 1000)))
   (setq *gzb-loop-flag* nil))
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; apply gazebo state to eus in nonblock loop
  (:gzb2eus-loop-nonblock
   (&optional model)

   (warning-message 2 "call (break-gzb2eus-loop-nonblock) to break loop~%")
   (unless (sys::free-threads)
     (sys:make-thread 1))
   (sys:thread #'send self :gzb2eus-loop model)
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; break from gzb2eus-loop-nonblock
  (:break-gzb2eus-loop-nonblock
   ()
   (setq *gzb-loop-flag* nil)
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; pause simulation
  (:pause-sim
   ()
   (call-empty-service "/gazebo/pause_physics")
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; unpause simulation
  (:unpause-sim
   ()
   (call-empty-service "/gazebo/unpause_physics")
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; reset world
  (:reset-world
   ()
   (call-empty-service "/gazebo/reset_world")
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; reset simulation
  (:reset-sim
   ()
   (call-empty-service "/gazebo/reset_sim")
   )
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; reset simulation
  (:model-list
   ()
   model-list)
  )

