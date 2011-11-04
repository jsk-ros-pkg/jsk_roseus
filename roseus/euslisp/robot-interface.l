;;;
;;; robot interface to ROS based pr2 system
;;;

(require :eustf "package://roseus/euslisp/eustf.l")
(require :actionlib "package://roseus/euslisp/actionlib.l")

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "pr2_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "pr2_controllers_msgs")

(defclass robot-interface
  :super propertied-object
  :slots (robot objects robot-state joint-action-enable
		controller-type controller-actions
		viewer))

(defmethod robot-interface
  (:init
   (&rest args &key ((:robot r)) ((:objects objs)) (type :default-controller) &allow-other-keys)
   (setq joint-action-enable t)
   (setq robot (instance r :init))
   ;;
   (defvar *tfl* (instance ros::transform-listener :init))
   (ros::subscribe "/joint_states" sensor_msgs::JointState
		   #'send self :ros-state-callback)

   (setq controller-type type)
   (mapcar
    #'(lambda (param)
	(let* ((controller (cdr (assoc :controller param)))
	       (action-type (cdr (assoc :action-type param)))
	       (action (instance ros::simple-action-client :init controller action-type)))
	  (push action controller-actions)))
    (send self controller-type))
   (nreverse controller-actions)

   (dolist (action controller-actions)
     (unless (and joint-action-enable (send action :wait-for-server 3))
       (setq joint-action-enable nil)
       (ros::ros-warn "~A is not respond, ~A-interface is disabled" action (send robot :name) )
       (return)))
   ;;
   (unless joint-action-enable
     (let ((old-viewer user::*viewer*))
       (when (and x::*display* (> x::*display* 0))
	 (setq viewer (instance x::irtviewer :create :title (format nil "~A-interface" (send robot :name))))
	 (send viewer :objects (list robot))
	 (send viewer :look-all)
	 (send self :objects objs)
	 (if old-viewer (setq user::*viewer* old-viewer)))

       (ros::advertise "/joint_states" sensor_msgs::JointState 1)
       ))
   self)
  ;;
  (:publish-joint-state ;; for simulation mode (joint-action-enable is nil)
   ()
   (let ((msg (instance sensor_msgs::JointState :init)) names positions velocities efforts)
     (send msg :header :stamp (ros::time-now))
     (dolist (j (send robot :joint-list))
       (push (string-downcase (symbol-name (send j :name))) names)
       (push (cond ((derivedp j rotational-joint)
		    (deg2rad (send j :joint-angle)))
		   (t
		    (* 0.001 (send j :joint-angle))))
	     positions)
       (push 0 velocities)
       (push 0 efforts))
     (send msg :name names)
     (send msg :position positions)
     (send msg :velocity velocities)
     (send msg :effort efforts)
     (unless joint-action-enable
       (ros::publish "/joint_states" msg))
     msg))
  (:angle-vector
   (av &optional (tm 3000) &key ((:controller-actions ca) controller-actions) ((:controller-type ct) controller-type))
   ;; for simulation mode
   (unless joint-action-enable
     (if av
       (let* ((prev-av (send robot :angle-vector))
	      (scale-av (send self :sub-angle-vector av prev-av)))
	 (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	     ((>= curr-tm tm))
	   (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
	   (send self :publish-joint-state)
	   (if viewer (send viewer :draw-objects))))))

   (send robot :angle-vector av)
   (mapcar
    #'(lambda (action param)
        (send self :send-ros-controller
              action (cdr (assoc :joint-names param)) ;; action server and joint-names
              0					      ;; start time
              (list
	       (list av				      ;; positions
		     (instantiate float-vector (length av))  ;; velocities
		     (/ tm 1000.0)))))			      ;; duration
	ca (send self ct))
   av)
  (:angle-vector-sequence
   (avs &optional (tms (list 3000)))
   (ros::spin-once) ;; for :state :potentio-vector
   (let ((st 0) (traj-points nil)
         (av-prev (send self :state :potentio-vector)) av av-next
         (offset (instantiate float-vector (length (send robot :angle-vector))))
         tm tm-next
         (vel (instantiate float-vector (length (car avs)))))
       (prog1 ;; angle-vector-sequence returns avs
	   avs
	 (while avs
	   (setq av (pop avs))
	   (when (= (car tms) 0.0)
	     (ros::ros-error "0.0 in time list @ :angle-vector-sequence"))
	   (if (car tms) (setq tm (max (pop tms) 1.0))) ;; minimum is 1[msec]
	   (if (setq av-next (car avs))
	       (let ((v0 (send self :sub-angle-vector av av-prev))
		     (v1 (send self :sub-angle-vector av-next av)))
		 (setq tm-next (max (if (car tms) (car tms) tm) 1.0))
		 (dotimes (i (length vel))
		   (setf (elt vel i)
			 (if (>= (* (elt v0 i) (elt v1 i)) 0)
			     (* 0.5 (+ (* (/ 1000.0 tm) (elt v0 i))
				       (* (/ 1000.0 tm-next) (elt v1 i))))
			   0.0)))
		 )
	     (fill vel 0))
	   ;; for simulation mode
	   (unless joint-action-enable
	     (let* ((prev-av (send robot :angle-vector))
		    (scale-av (send self :sub-angle-vector av prev-av)))
	       (do ((curr-tm 0.0 (+ curr-tm 100.0)))
		   ((>= curr-tm tm))
		 (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
		 (send self :publish-joint-state)
		 (if viewer (send viewer :draw-objects)))))
	   ;;
	   (send robot :angle-vector av)

	   ;; add offset for rotational joint
	   ;; eus: -180 - 180
	   ;; joit trajectory points : -inf - + inf
	   (when av-prev
	     (let ((joint-list (send robot :joint-list)) (av-diff (v- av av-prev)) (i 0) j)
	       (while (setq j (pop joint-list))
		 (when (and (= (send j :min-angle) *-inf*) (= (send j :max-angle) *inf*))
		   (cond ((> (elt av-diff i)  180.0)
			  (incf (elt offset i) -360.0))
			 ((< (elt av-diff i) -180.0)
			  (incf (elt offset i)  360.0)
			  )))
		 (incf i)
		 )))
	   (push (list (v+ av offset)  ;; positions
		       (copy-seq vel)  ;; velocities
		       (+ (/ (+ st tm) 1000.0) 0.2)) ;; tm + duration
		 traj-points)
	   (setq av-prev av)
	   (incf st tm))
	 ;;
	 (mapcar
	  #'(lambda (action param)
	      (send self :send-ros-controller
		    action (cdr (assoc :joint-names param)) ;; action server and joint-names
		    0.1   ;; start time
		    traj-points))
	  controller-actions (send self controller-type))
	 )))
  (:wait-interpolation
   ()
   (unless joint-action-enable (return-from :wait-interpolation nil))
   (send-all controller-actions :wait-for-result))
  ;;
  ;;
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (unless joint-action-enable (return-from :send-ros-controller nil))
   (let* ((goal (send action :make-goal-instance))
	  (goal-points nil)
	  (st (ros::time+ (ros::time-now) (ros::time starttime)))
	  (joints (mapcar #'(lambda (x)
			      (send robot (intern (string-upcase x) *keyword-package*)))
			  joint-names)))
     (send goal :header :seq 1)
     (send goal :header :stamp st)
     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp st)
     (dolist (trajpt trajpoints)
       (let* ((all-positions (elt trajpt 0))
	      (all-velocities (elt trajpt 1))
	      (duration (elt trajpt 2))
	      (positions (instantiate float-vector (length joint-names)))
	      (velocities (instantiate float-vector (length joint-names))))
	 (dotimes (i (length joints))
	   (let* ((joint (elt joints i))
		  (id (position joint (send robot :joint-list)))
		  p v)
	     (setq p (elt all-positions id)
		   v (elt all-velocities id))
	     (cond
	      ((derivedp joint rotational-joint)
	       (setq p (deg2rad p))
	       (setq v (deg2rad v)))
	      (t
	       (setq p (* 0.001 p))
	       (setq v (* 0.001 v))))
	     (setf (elt positions i) p)
	     (setf (elt velocities i) v)))
	 (push (instance trajectory_msgs::JointTrajectoryPoint
			 :init
			 :positions positions
			 :velocities velocities
			 :time_from_start (ros::time duration))
	       goal-points)
	 ))
     (ros::spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send action :send-goal goal)
     ))
  ;;
  (:ros-state-callback
   (msg)
   (dolist (key '(:name :position :velocity :effort))
     (if (assoc key robot-state)
	 (setf (cdr (assoc key robot-state)) (send msg key))
       (push (cons key (send msg key)) robot-state))))
  (:update-robot-state
   ()
   (let (joint-names positions velocities efforts)
     (unless joint-action-enable
       (return-from :update-robot-state (send robot :angle-vector)))
     (unless robot-state (return-from :update-robot-state))
     (ros::spin-once)
     (setq joint-names (cdr (assoc :name robot-state))
	   positions (cdr (assoc :position robot-state))
	   velocities (cdr (assoc :velocity robot-state))
	   efforts (cdr (assoc :effort robot-state)))
     (let ((joint-num (length joint-names)))
       (when (not (eq joint-num (length velocities)))
	 (setq velocities (instantiate float-vector joint-num)))
       (when (not (eq joint-num (length efforts)))
	 (setq efforts (instantiate float-vector joint-num))))
     (mapcar #'(lambda (n p v e)
		 (let (j (kn (intern (string-upcase n) *keyword-package*)))
		   (when (and (find-method robot kn) (setq j (send robot kn)))
		     (cond
		      ;; position
		      ((derivedp j rotational-joint)
		       (send j :joint-angle (rad2deg p)))
		      (t
		       (send j :joint-angle (* 1000 p))))
			 ;; velocity
		     (send j :joint-velocity v)
		     ;; effort
		     (send j :joint-torque e))))
	     (coerce joint-names cons)
	     (coerce positions cons)
	     (coerce velocities cons)
	     (coerce efforts cons))))
  (:state
   (&rest args)
   (case (car args)
     ((:angle-vector nil)
      (ros::ros-warn "please use :state :potentio-vector~%")
      (send self :state :potentio-vector))
     (:potentio-vector
      (send self :update-robot-state)
      (send robot :angle-vector))
     (:torque-vector
      (send self :update-robot-state)
      (send robot :torque-vector))
     (:worldcoords
      (send *tfl* :lookup-transform (or (cadr args) "/map") "/base_footprint" (ros::time)))
     (t
      (let ((cur robot-state))
	(dolist (key args cur)
	  (setq cur (cdr (assoc key cur))))))
     ))
  ;;
  (:default-controller
   ()
   (list
    (list
     (cons :controller "fullbody_controller/joint_trajectory_action")
     (cons :action-type pr2_controllers_msgs::JointTrajectoryAction)
     (cons :joint-names (mapcar #'(lambda (n) (symbol-name n)) (send-all (send robot :joint-list) :name))))))
  ;;
  (:sub-angle-vector (v0 v1)
   (let ((ret (v- v0 v1))
	 (joint-list (send robot :joint-list))
	 (i 0) j)
     (while (setq j (pop joint-list))
       (if (and (= (send j :min-angle) *-inf*) (= (send j :max-angle) *inf*))
	   (cond ((> (elt ret i) 180.0)
		  (setf (elt ret i) (- (elt ret i) 360.0)))
		 ((< (elt ret i) -180.0)
		  (setf (elt ret i) (+ (elt ret i) 360.0)))))
       (incf i))
     ret))
  ;;
  (:robot  (&rest args) (forward-message-to robot args))
  (:viewer (&rest args) (forward-message-to viewer args))
  (:objects (&optional objs)
	    (when objs
	      (setq objects (mapcar #'(lambda (o) (let ((p (send o :parent))) (if p (send p :dissoc o))(copy-object o)(if p (send p :assoc o)))) objs))
	      (mapcar #'(lambda (o) (setf (get o :GL-DISPLAYLIST-ID) nil)) (x::draw-things objects))
	      (if viewer (send viewer :objects (append (list robot) objects))))
	    objects)
  ) ;; ros-interface


;; 
(defclass ros-interface
  :super robot-interface
  :slots ())
(defmethod ros-interface
  (:init
   (&rest args)
   (ros::ros-error "please use robot-interface class, ros-interface class will be removed")
   (send-super* :init args))

(provide :robot-interface "robot-interface.l")

