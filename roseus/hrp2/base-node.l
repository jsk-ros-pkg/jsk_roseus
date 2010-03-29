;; base-node.l
;; this file makes a ros-node to take the public /move_base_simple/goal
;; and exec go-pos toward goal

;; ...
(defvar *real* nil)
(defvar *go-pos-debug* t)

;; donot change!
(defvar *ros-lock* (sys::make-mutex-lock))

;; bootstrap
;; roeus
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")

;; hrp2
(load (format nil "~A/hrp2/corba/orbixif.l" (unix::getenv "CVSDIR")))
(load (format nil "~A/hrp2/plugins/plugins.l" (unix::getenv "CVSDIR")))

;; ros messages
(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "geometry_msgs")

(defun warn* (context str &rest args)
  "this function outputs a string to standard error.
example:
  (warn-message :info \"this is a test\")
=> [INFO] this is a test"
  (warning-message 1 "[~A] " (string context)) ;tricky...
  (apply #'warn (format nil "~A~%" str) args))

(defun go-pos-callback (msg)
  "ros callback function. DONT call this function from lisp top-level."
  (if *go-pos-debug* (warn* :info "go-pos-callback is called with ~A" msg))
  ;; msg = geometry_msgs::PoseStamped
  ;; geometry_msgs::PoseStamped = <header pose>
  ;; Pose = <position orientation>
  ;; position = <x y z>
  (let* ((pose (send msg :pose))
         (position (send pose :position))
         (orientation (send pose :orientation)))
    (let ((x (send position :x))
          (y (send position :y))
          (z (send position :z)))
      ;;NB: x, y, z are relative values?
      ;;NB: x, y, z's unit is [mm]?
      (when *go-pos-debug*
        (warn* :info "x: => ~A" x)
        (warn* :info "y: => ~A" y)
        (warn* :info "z: => ~A" z))
      ))
  (when *real*
    (ros::ros-info "send go-pos ~A ~A ~A" (* 0.001 x) (* 0.001 y) 0)
    (send *ci* :go-pos (* 0.001 x) (* 0.001 y) 0))
  t)

;; initialize function
(defun init ()
  "initialize function"
  ;; format
  (if *real*
      (ros::ros-info "base-node.l works in non-real mode")
    (ros::ros-info "base-node.l works in real mode"))
  (ros::ros-info "you can use /move_base_simple/goal to move hrp2!")
  (ros::ros-info "the value must be relative and in mm")
  (ros::ros-info "HRP2-> ~A" (unix:getenv "ROBOT"))
  ;; hrp2 initialization
  (when *real*
    (setq *hrp2* (case (intern (unix:getenv "ROBOT"))
                   (hrp2jsk
                    (hrp2jsk-simple-detail))
                   (hrp2jsknt
                    (hrp2jsknt-simple-detail))
                   (hrp2w
                    (hrp2jskw-simple-detail))))
    (setq *ci* (eusorbix-init)))
  ;; ros initialization
  (roseus "clear")
  ;; set ros callback for "/move_base_simple/goal"
  (ros::subscribe "/move_base_simple/goal" geometry_msgs::PoseStamped #'go-pos-callback)
  (ros::init-ros-wakeup-worker-thread)
  )

;; main function
(defun main ()
  )

;; (load "base-node.l")
;; (init)
;; (main)

