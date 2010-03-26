;; base-node.l
;; this file makes a ros-node to take the public /move_base_simple/goal
;; and exec go-pos toward goal

;;
(defvar *real* nil)
(defvar *go-pos-debug* t)

;; bootstrap
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)

(load "roseus.l")

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
  "ros callback function"
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
      (when *go-pos-debug*
        (warn* :info "x: => ~A" x)
        (warn* :info "y: => ~A" y)
        (warn* :info "z: => ~A" z))
      ))
  t)

;; initialize function
(defun init ()
  ;; ros initialization
  (roseus "clear")
  ;; set ros callback for "/move_base_simple/goal"
  (ros::subscribe "/move_base_simple/goal" geometry_msgs::PoseStamped
    #'go-pos-callback)
  (ros::init-ros-wakeup-worker-thread)
  )

;; main function
(defun main ()
  )

;; (load "base-node.l")
;; (init)
;; (main)

