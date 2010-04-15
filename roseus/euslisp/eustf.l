;; this is a collection of utility functions and macros
;; for euslisp coordination system interface and
;; ros coordination system interface

;; load c-binding library
(load-library "eustf.so" '("eustf"))

;; include ros messages packages
(roseus-add-msgs "roslib")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "geometry_msgs")

;; convert unit....
;; position convert : WG use [m], JSK use eus [mm]
;; quaternion convert: WG use axis-angle quaternion, JSK use angle-axis quaternion

;; eus->tf
(defun ros::pos->tf-point (pos)
  (instance geometry_msgs::point :init
            :x (* 0.001 (x pos)) :y (* 0.001 (y pos)) :z (* 0.001 (z pos))))

(defun ros::rot->tf-quaternion (coords)
  (let* ((q (matrix2quaternion rot))
         (qx (elt q 3)) (qy (elt q 1)) (qz (elt q 2)) (qw (elt q 0)))
    (instance geometry_msgs::quaternion :init
              :x qx :y qy :z qz :w qw)))

(defun ros::coords->tf-pose (coords)
  (instance geometry_msgs::pose :init
            :position (ros::pos->tf-position (send coords :worldpos))
            :orientation (ros::rot->tf-quaternion (send coords :worldrot))))

(defun ros::coords->tf-pose-stamped (coords id)
  (let ((pose (ros::coords->tf-pose coords)))
    ;; PoseStamped = Header + Pose
    ;; Header = seq + frame_id
    (instance geometry_msgs::PoseStamped :init
              :header (instance roslib::header :init
                                :frame_id id
                                :stamp (ros::time-now))
              :pose pose)))

(defun ros::coords->tf-transform (coords) (ros::coords->tf-pose coords))

;; tf->eus
(defun ros::tf-point->pos (point)
  (float-vector (* 1000.0 (send point :x)) (* 1000.0 (send point :y))  (* 1000.0 (send point :z))))

(defun ros::tf-quaternion->rot (quaternion)
  (quaternion2matrix
   (float-vector (send quaternion :x) (send quaternion :y) (send quaternion :z) (send quaternion :w))))

(defun ros::tf-pose->coords (pose)
  (make-coords (ros::tf-point->pos (send pose :position))
               (ros::tf-quaternion->rot (send pose :orientation))))

(defun ros::tf-pose-stamped->coords (pose-stamped)
  (make-cascoords :coords (ros::tf-pose->coords (send pose-stamped :pose))
                  :name (send pose-stamped :header :frame_id)))

(defun ros::tf-transform->coords (pose) (ros::tf-pose->coords pose))

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
  (let* ((q (ros::create-quaternion-from-rpy roll pitch yaw))
         (qx (elt q 3)) (qy (elt q 1)) (qz (elt q 2)) (qw (elt q 0)))
    (instance geometry_msgs::quaternion :init :x qx :y qy :z qz :w qw)))

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
   (setq cobject (ros::eustf-transformer interpolating cache-time))
   self)
  (:all-frames-as-string
   ()
   (ros::eustf-all-frames-as-string cobject))
  (:set-transform
   (trans &optional (auth ""))
   (ros::eustf-set-transform cobject))
  (:wait-for-transform
   (target-frame source-frame time timeout &optional (duration 0.01))
   (ros::eustf-wait-for-transform cobject
                                  target-frame source-frame
                                  (send time :sec-nsec) timeout duration))
  (:wait-for-transform-full
   (target-frame target-time source-frame source-time fixed-frame timeout &optional (duration 0.01))
   (ros::eustf-wait-for-transform-full cobject
                                       target-frame (send target-time :sec-nsec)
                                       source-frame (send source-time :sec-nsec)
                                       fixed-frame timeout duration))
  (:can-transform
   (tareget-frame source-frame time)
   (ros::eustf-can-transform cobject
                             target-frame source-frame (send time :sec-nsec)))
  (:can-transform-full
   (tareget-frame target-time source-frame source-time fixed-frame)
   (ros::eustf-can-transform-full cobject
                                  target-frame (send target-time :sec-nsec)
                                  source-frame (send source-time :sec-nsec)
                                  fixed-frame))
  (:chain
   (target-frame target-time source-frame source-time fixed-frame)
   (ros::eustf-chain cobject))
  (:clear () (ros::eustf-clear cobject))
  (:frame-exists (frame-id) (ros::eustf-all-frames-as-string cobject frame_id))
  (:get-frame-strings () (ros::eustf-get-frame-strings cobject))
  (:get-latest-common-time
   (source-frame target-frame)
   (ros::eustf-get-latest-common-time cobject source-frame target-frame))
  (:lookup-transform
   (target-frame source-frame time)
   (let (ret)
     (setq ret (ros::eustf-lookup-transform cobject
                                            target-frame source-frame
                                            (send time :sec-nsec)))
     (make-coords :pos (float-vector
                        (* 1000.0 (elt ret 0))
                        (* 1000.0 (elt ret 1))
                        (* 1000.0 (elt ret 2)))
                  :rot
                  (quaternion2matrix
                   (float-vector (elt ret 3)
                                 (elt ret 4)
                                 (elt ret 5)
                                 (elt ret 6)))
                  :name target-frame)))
  (:lookup-transform-full
   (target-frame target-time source-frame source-time fixed-frame)
   (let (ret)
     (setq ret (ros::eustf-lookup-transform-full cobject
                                                 target-frame (send target-time :sec-nsec)
                                                 source-frame (send source-time :sec-nsec)
                                                 fixed-frame))
     (make-coords :pos (float-vector
                        (* 1000.0 (elt ret 0))
                        (* 1000.0 (elt ret 1))
                        (* 1000.0 (elt ret 2)))
                  :rot
                  (quaternion2matrix
                   (float-vector (elt ret 3)
                                 (elt ret 4)
                                 (elt ret 5)
                                 (elt ret 6)))
                  :name target-frame)))
  (:lookup-velocity
   (reference-frame moving-frame time duration)
   (ros::eustf-lookup-velocity cobject reference-frame moving-frame time duration))
  (:get-parent
   (frame_id time)
   (ros::eustf-get-parent cobject frame_id (send time :sec-nsec)))
  ;;
  (:set-extrapolation-limit
   (distance)
   (ros::eustf-set-extrapolation-limit cobject distance))
  )

(defclass ros::transform-listener
  :super ros::transformer
  :slots ())
(defmethod ros::transform-listener
  (:init
   (&optional (cache-time 10.0) (spin-thread t))
   (setq cobject (ros::eustf-transform-listener cache-time spin-thread))
   self)
  )
