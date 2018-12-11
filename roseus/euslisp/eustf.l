;; this is a collection of utility functions and macros
;; for euslisp coordination system interface and
;; ros coordination system interface

(in-package "ROS")
(use-package "GEOMETRY")

;; load c-binding library
(load-library "eustf.so" '("eustf"))

;; include ros messages packages
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(defclass tf-cascaded-coords
  :super cascaded-coords
  :slots (frame-id child-frame-id stamp))
(defmethod tf-cascaded-coords
  (:init (&rest initargs &key ((:frame-id fid) "") ((:child-frame-id cid) "") ((:stamp st) #i(0 0)) &allow-other-keys)
    (send-super* :init initargs)
    (setf frame-id fid child-frame-id cid stamp st)
    self)
  (:frame-id (&optional id) (if id (setq frame-id id)) frame-id)
  (:child-frame-id (&optional id) (if id (setq child-frame-id id)) child-frame-id)
  (:stamp (&optional st) (if st (setq stamp st)) stamp)
  (:prin1 (&optional (strm 1))
          (send-super :prin1 strm (format nil "~s ~s ~d.~d" frame-id child-frame-id (elt stamp 0) (elt stamp 1))))
  )



;; convert unit....
;; position convert : WG use [m], JSK use eus [mm]
;; quaternion convert: WG use axis-angle quaternion, JSK use angle-axis quaternion

;; eus->tf
(defun ros::pos->tf-point (pos)
  "Convert euslisp pos to geoometry_msgs::Point, this also converts [mm] to [m]"
  (instance geometry_msgs::point :init
            :x (* 0.001 (elt pos 0)) :y (* 0.001 (elt pos 1)) :z (* 0.001 (elt pos 2))))
(defun ros::pos->tf-translation (pos)
  "Convert euslisp pos to geoometry_msgs::Vector3, this also converts [mm] to [m]"
  (instance geometry_msgs::vector3 :init
            :x (* 0.001 (elt pos 0)) :y (* 0.001 (elt pos 1)) :z (* 0.001 (elt pos 2))))

(defun ros::rot->tf-quaternion (rot)
  "Convert euslisp rot to geoometry_msgs::Quaternion"
  (let* ((q (user::matrix2quaternion rot))
         (qx (elt q 1)) (qy (elt q 2)) (qz (elt q 3)) (qw (elt q 0)))
    (instance geometry_msgs::quaternion :init
              :x qx :y qy :z qz :w qw)))

(defun ros::coords->tf-pose (coords)
  "Convert euslisp coordinates to geoometry_msgs::Pose"
  (instance geometry_msgs::pose :init
            :position (ros::pos->tf-point (send coords :worldpos))
            :orientation (ros::rot->tf-quaternion (send coords :worldrot))))

(defun ros::coords->tf-pose-stamped (coords id)
  "Convert euslisp coordinates to geoometry_msgs::PoseStapmed"
  (let ((pose (ros::coords->tf-pose coords)))
    ;; PoseStamped = Header + Pose
    ;; Header = seq + frame_id
    (instance geometry_msgs::PoseStamped :init
              :header (instance std_msgs::header :init
                                :frame_id id
                                :stamp (ros::time-now))
              :pose pose)))

(defun ros::coords->tf-transform (coords)
  "Convert euslisp coordinates to geoometry_msgs::Transform"
  (instance geometry_msgs::transform :init
            :translation (ros::pos->tf-translation (send coords :worldpos))
            :rotation (ros::rot->tf-quaternion (send coords :worldrot))))

(defun ros::coords->tf-transform-stamped (coords id &optional (child_id ""))
  "Convert euslisp coordinates to geoometry_msgs::TransformStamped"
    (let ((trans (ros::coords->tf-transform coords)))
      (instance geometry_msgs::TransformStamped :init
                :header (instance std_msgs::header :init :frame_id id :stamp (ros::time-now))
                :child_frame_id child_id :transform trans)))

;; tf->eus
(defun ros::tf-point->pos (point)
  "Convert geoometry_msgs::Point to euslisp point vector, this aloso converts [m] to [mm]"
  (float-vector (* 1000.0 (send point :x)) (* 1000.0 (send point :y))  (* 1000.0 (send point :z))))

(defun ros::tf-quaternion->rot (quaternion)
  "Convert geoometry_msgs::quaternion to euslisp rotation matrix"
  (user::quaternion2matrix
   (float-vector (send quaternion :w)
                 (send quaternion :x)
                 (send quaternion :y) 
                 (send quaternion :z))))

(defun ros::tf-pose->coords (pose)
  "Convert geoometry_msgs::Pose to euslisp coordinates"
  (make-coords :pos (ros::tf-point->pos (send pose :position))
               :rot (ros::tf-quaternion->rot (send pose :orientation))))

(defun ros::tf-pose-stamped->coords (pose-stamped)
  "Convert geoometry_msgs::PoseStamped to euslisp coordinates"
  (make-cascoords :coords (ros::tf-pose->coords (send pose-stamped :pose))
                  :name (send pose-stamped :header :frame_id)))

(defun ros::tf-transform->coords (trans)
  "Convert geoometry_msgs::Transform to euslisp coordinates"
  ;; check if trans class is geometry_msgs::TransformStamped
  ;; ros::tf-transform->coords was wrongly implemented to take TransformStamped as argument from electric to kinetic
  ;; Thus, to keep backward compatibility, we should allow users to pass both Transform and TransformStamped
  (if (derivedp trans geometry_msgs::Transform)
      (instance tf-cascaded-coords
                :init
                :pos (ros::tf-point->pos (send trans :translation))
                :rot (ros::tf-quaternion->rot (send trans :rotation)))
  (instance tf-cascaded-coords
            :init
            :pos (ros::tf-point->pos (send trans :transform :translation))
            :rot (ros::tf-quaternion->rot (send trans :transform :rotation))
            :frame-id (send trans :header :frame_id)
            :child-frame-id (send trans :child_frame_id)
            :stamp (send (send trans :header :stamp) :sec-nsec)))
  )

(defun ros::tf-transform-stamped->coords (trans)
  "Convert geoometry_msgs::TransformStamped to euslisp coordinates"
  (ros::tf-transform->coords trans))

(defun ros::tf-twist->coords (twist)
  "Convert geoometry_msgs::Twist to euslisp coordinates"
  (let* ((pos (ros::tf-point->pos (send twist :linear)))
         (ang (send twist :angular))
         (rot (user::matrix-exponent (float-vector (send ang :x) (send ang :y) (send ang :z))))
         (cds (make-coords :pos pos :rot rot)))
    cds))

;; helper functions
(defun ros::create-identity-quaternion () (ros::create-quaternion-from-rpy 0 0 0))
(defun ros::create-quaternion-from-rpy (roll pitch yaw)
  (let ((sin-roll  (sin (* roll  0.5))) (cos-roll  (cos (* roll  0.5)))
        (sin-pitch (sin (* pitch 0.5))) (cos-pitch (cos (* pitch 0.5)))
        (sin-yaw   (sin (* yaw   0.5))) (cos-yaw   (cos (* yaw   0.5))))
    (print (list sin-roll cos-roll
                 sin-pitch cos-pitch
                 sin-yaw cos-yaw))
    (float-vector (- (* sin-roll cos-pitch cos-yaw) (* cos-roll sin-pitch sin-yaw))
                  (+ (* cos-roll sin-pitch cos-yaw) (* sin-roll cos-pitch sin-yaw))
                  (- (* sin-roll cos-pitch sin-yaw) (* sin-roll sin-pitch cos-yaw))
                  (+ (* cos-roll cos-pitch cos-yaw) (* sin-roll sin-pitch sin-yaw)))))

(defun ros::create-quaternion-msg-from-rpy (roll pitch yaw)
  "Create geoometry_msgs::Quaternion from roll, pitch and yaw"
  (let* ((q (ros::create-quaternion-from-rpy roll pitch yaw))
         (qx (elt q 1)) (qy (elt q 2)) (qz (elt q 3)) (qw (elt q 0)))
    (instance geometry_msgs::quaternion :init :x qx :y qy :z qz :w qw)))

(defun ros::identity-quaternion () "Create identity geoemtry_msgs::Quaternion" (instance geometry_msgs::quaternion :init :w 1))
(defun ros::identity-pose () "Create identity geometry_msgs::Pose" (instance geometry_msgs::Pose :init
                                       :orientation (ros::identity-quaternion)))
(defun ros::identity-pose-stamped (&optional (header (instance std_msgs::header :init)))
  "Create identity geometry_msgs::PoseStamped"
  (instance geometry_msgs::PoseStamped :init
            :header header
            :pose (ros::identity-pose)))
(defun ros::identity-transform ()
  "Create identity geometry_msgs::Transform"
  (instance geometry_msgs::transform :init
            :rotation (ros::identity-quaternion)))
(defun ros::identity-transform-stamped (&optional (header (instance std_msgs::header :init)))
  "Create identity geometry_msgs::TransformStamped"
  (instance geometry_msgs::TransformStamped :init
            :header header
            :transform (ros::identity-transform)))

;;
;; transfromer interface
;; http://www.ros.org/doc/api/tf/html/python/tf_python.html#transformer
;;
(defclass ros::transformer
  :super ros::object
  :slots (cobject))
(defmethod ros::transformer
  (:init
   (&optional (interpolating t) (cache-time 10.0))
   "The Transformer object is the core of tf. It maintains a time-varying graph of transforms, and permits asynchronous graph modification and queries

$ (setq tr (instance ros::transformer :init))
#<ros::transformer #X545e290>
$ (send tr :set-transform (instance ros::tf-cascaded-coords :init :pos #f(1 2 3) :frame-id \"this_frame\" :child-frame-id \"child\"))
$ (send tr :get-frame-strings)
(\"this_frame\" \"child\")
$ (send tr :lookup-transform \"this_frame\" \"child\" (ros::time))
#<cascaded-coords #X5438430 this_frame  1000.0 2000.0 3000.0 / 3.142 0.0 3.142>
$ (send tr :lookup-transform \"child\"  \"this_frame\" (ros::time))
#<cascaded-coords #X54388e0 child  1000.0 -2000.0 3000.0 / 3.142 0.0 3.142>
"
   (setq cobject (ros::eustf-transformer interpolating cache-time))
   self)
  (:all-frames-as-string
   ()
   "Returns a string representing all frames"
   (ros::eustf-all-frames-as-string cobject))
  (:set-transform
   (coords &optional (auth ""))
   "Adds a new transform to the Transformer graph from euslisp coordinates object"
   (let ((pos (send coords :pos))
         (rot (user::matrix2quaternion (send coords :rot))))
     (if (not (derivedp coords tf-cascaded-coords))
         (warn ";; mismatch argument set-transform [tf-cascaded-coords]"))
     (ros::eustf-set-transform cobject pos rot (send coords :stamp)
                               (send coords :frame-id)
                               (send coords :child-frame-id) auth)))
  (:wait-for-transform
   (target-frame source-frame time timeout &optional (duration 0.01))
   "Waits for the given transformation to become available. If the timeout occurs before the transformation becomes available"
   (ros::eustf-wait-for-transform cobject
                                  target-frame source-frame
                                  (send time :sec-nsec) timeout duration))
  (:wait-for-transform-full
   (target-frame target-time source-frame source-time fixed-frame timeout &optional (duration 0.01))
   "Extended version of :wait-for-transform"
   (ros::eustf-wait-for-transform-full cobject
                                       target-frame (send target-time :sec-nsec)
                                       source-frame (send source-time :sec-nsec)
                                       fixed-frame timeout duration))
  (:can-transform
   (target-frame source-frame time)
   "Returns True if the Transformer can determine the transform from source_frame to target_frame at time"
   (ros::eustf-can-transform cobject
                             target-frame source-frame (send time :sec-nsec)))
  (:can-transform-full
   (target-frame target-time source-frame source-time fixed-frame)
   "Extended version of can-transform"
   (ros::eustf-can-transform-full cobject
                                  target-frame (send target-time :sec-nsec)
                                  source-frame (send source-time :sec-nsec)
                                  fixed-frame))
  (:chain
   (target-frame target-time source-frame source-time fixed-frame)
   "returns the chain of frames connecting source_frame to target_frame."
   (ros::eustf-chain cobject))
  (:clear () "Clear all transformations." (ros::eustf-clear cobject))
  (:frame-exists (frame-id) "returns True if frame frame_id exists in the Transformer." (ros::eustf-frame-exists cobject frame-id))
  (:get-frame-strings () "returns all frame names in the Transformer as a list" (ros::eustf-get-frame-strings cobject))
  (:get-latest-common-time
   (source-frame target-frame)
   "Determines the most recent time for which Transformer can compute the transform between the two given frames. Raises tf.Exception if transformation is not possible."
   (let ((sec-nsec
          (ros::eustf-get-latest-common-time cobject source-frame target-frame)))
     (if sec-nsec
         (instance ros::time :init
                   :sec (car sec-nsec)
                   :nsec (cdr sec-nsec)))))
  (:lookup-transform
   (target-frame source-frame time)
   "Returns the transform from source_frame to target_frame at time."
   (let (ret)
     (setq ret (ros::eustf-lookup-transform cobject
                                            target-frame source-frame
                                            (send time :sec-nsec)))
     (when ret
       (make-cascoords :pos (float-vector
                             (* 1000.0 (elt ret 0))
                             (* 1000.0 (elt ret 1))
                             (* 1000.0 (elt ret 2)))
                       :rot
                       (user::quaternion2matrix
                        (float-vector (elt ret 3)
                                      (elt ret 4)
                                      (elt ret 5)
                                      (elt ret 6)))
                       :name target-frame))))
  (:lookup-transform-full
   (target-frame target-time source-frame source-time fixed-frame)
   "Extended version of :lookup-transform"
   (let (ret)
     (setq ret (ros::eustf-lookup-transform-full cobject
                                                 target-frame (send target-time :sec-nsec)
                                                 source-frame (send source-time :sec-nsec)
                                                 fixed-frame))
     (when ret
       (make-cascoords :pos (float-vector
                             (* 1000.0 (elt ret 0))
                             (* 1000.0 (elt ret 1))
                             (* 1000.0 (elt ret 2)))
                       :rot
                       (user::quaternion2matrix
                        (float-vector (elt ret 3)
                                      (elt ret 4)
                                      (elt ret 5)
                                      (elt ret 6)))
                       :name target-frame))))
  (:lookup-velocity
   "Return the linear and angular velocity of the moving_frame in the reference_frame. tf considers time - duration / 2 to time + duration / 2 as the initial interval, and will shift by up to duration / 2 to avoid no data."
   (reference-frame moving-frame time duration)
   (ros::eustf-lookup-velocity cobject reference-frame moving-frame time duration))
  (:get-parent
   (frame_id time)
   "Fill the parent of a frame."
   (ros::eustf-get-parent cobject frame_id (send time :sec-nsec)))
  ;;
  (:set-extrapolation-limit
   (distance)
   "Set the distance which tf is allow to extrapolate."
   (ros::eustf-set-extrapolation-limit cobject distance))
  )

(defclass ros::transform-listener
  :super ros::transformer
  :slots ())
(defmethod ros::transform-listener
  (:init
   (&optional (cache-time 10.0) (spin-thread t))
   "This class inherits from Transformer and automatically subscribes to ROS transform messages."
   (sys::dispose-hook self t)
   (setq cobject (ros::eustf-transform-listener cache-time spin-thread))
   self)
  (:dispose
   ()
   (ros::eustf-transform-listener-dispose cobject))
  (:transform-pose
   (target-frame pose-stamped)
   "Transform a geometry_msgs::PoseStamped into the target frame"
   (let ((pose (float-vector (send pose-stamped :pose :position :x)
                             (send pose-stamped :pose :position :y)
                             (send pose-stamped :pose :position :z)
                             (send pose-stamped :pose :orientation :w)
                             (send pose-stamped :pose :orientation :x)
                             (send pose-stamped :pose :orientation :y)
                             (send pose-stamped :pose :orientation :z))))
     (let ((ret (ros::eustf-transform-pose cobject
					   target-frame
					   (send (send pose-stamped :header :stamp) :sec-nsec)
					   (send pose-stamped :header :frame_id)
					   pose)))
       (when ret
         (make-cascoords :pos (float-vector
                               (* 1000.0 (elt ret 0))
                               (* 1000.0 (elt ret 1))
                               (* 1000.0 (elt ret 2)))
                         :rot (user::quaternion2matrix (float-vector (elt ret 3)
                                                                     (elt ret 4)
                                                                     (elt ret 5)
                                                                     (elt ret 6)))
                         :name target-frame)))))
  )

(defclass ros::transform-broadcaster
  :super ros::object
  :slots (cobject)
  )
(defmethod ros::transform-broadcaster
  (:init () "Construnct transform broadcaster" (setq cobject (ros::eustf-transform-broadcaster)))
  (:send-transform
   (coords p-frame c-frame &optional tm)
   "Send a StampedTransform from euslisp coordinates, parent frame, frame_id, and time"
   (let ((pos (send coords :worldpos))
         (quat (user::matrix2quaternion (send coords :worldrot)))
         (stm (cond ((derivedp tm ros::time) (send tm :sec-nsec))
		    ((integer-vector-p tm) tm)
		    (t (send (ros::time-now) :sec-nsec))))
         )
     (ros::eustf-send-transform cobject pos quat p-frame c-frame stm)
     ))
  )

(defclass ros::buffer-client
  :super ros::object
  :slots (cobject)
  )
(defmethod ros::buffer-client
  (:init (&key (namespace "tf2_buffer_server") (check-frequency 10.0)
               (timeout-padding 2.0))
   "Construct tf2 buffer client object.
Keyword Args:
    namespace (str): The namespace in which to look for a tf2_ros::BufferServer (default: tf2_buffer_server)
    check-frequency (float): The frequency in Hz to check whether the BufferServer has completed a request (default: 10.0)
    timeout-padding (float): The amount of time to allow passed the desired timeout on the client side of communication lag (default: 2.0)
"
   (setq cobject (ros::eustf-buffer-client namespace check-frequency timeout-padding))
   self)
  (:wait-for-server (&optional (timeout 5.0))
   "Wait for tf2 buffer server"
   (if timeout (eustf-tf2-wait-for-server cobject timeout)
     (eustf-tf2-wait-for-server cobject)))
  (:wait-for-transform ;; this method does not exist in c++
   (target-frame source-frame rostime timeout &optional (duration 0.05))
   "Waits for the given transformation to become available. If the timeout occurs before the transformation becomes available"
   (let ((tm-ed (ros::time+ (ros::time-now) (ros::time timeout)))
         (tm-out (ros::time duration))
         ret)
     (while
         (and (null
               (setq ret (send self :can-transform target-frame source-frame rostime duration)))
              (> (send (ros::time- tm-ed (ros::time-now)) :to-sec) 0)))
     ret))
  (:can-transform (target-frame source-frame rostime &optional (timeout 0.0))
   "Returns t if the Transformer can determine the transform from source_frame to target_frame at time."
   (ros::eustf-tf2-can-transform
    cobject target-frame source-frame (send rostime :sec-nsec) timeout))
  (:lookup-transform (target-frame source-frame rostime &optional (timeout 0.0))
   "Returns the transform from source_frame to target_frame at time."
   (let (ret)
     (setq ret (ros::eustf-tf2-lookup-transform
                cobject target-frame source-frame
                (send rostime :sec-nsec) timeout))
     (when ret
       (make-cascoords :pos (float-vector
                             (* 1000.0 (elt ret 0))
                             (* 1000.0 (elt ret 1))
                             (* 1000.0 (elt ret 2)))
                       :rot
                       (user::quaternion2matrix
                        (float-vector (elt ret 3)
                                      (elt ret 4)
                                      (elt ret 5)
                                      (elt ret 6)))
                       :name target-frame))))
  )
(provide :eustf "eustf.l")
