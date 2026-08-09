#!/usr/bin/env roseus
;;;
;;; Regression test server for CDR serialization of nested messages that
;;; contain EMPTY dynamic arrays.
;;;
;;; A FollowJointTrajectory goal carries a JointTrajectoryPoint whose
;;; `positions` array is non-empty but whose `velocities`/`accelerations`/
;;; `effort` arrays are empty. rosidl_generator_eus must not emit
;;; element-alignment padding after the length of an empty sequence; if it
;;; does, rmw (Fast CDR) fails to deserialize the goal request and the client
;;; never reaches this server.
;;;
;;; This server decodes the received goal and echoes what it read back through
;;; the result so the client can assert a correct round-trip:
;;;   error_code   = number of positions in the first trajectory point
;;;   error_string = the first position value (as decoded on the server)
;;;

(ros::load-ros-package "control_msgs")
(require :actionlib2 "actionlib2.l")

(ros::roseus "cdr_nested_empty_array_server")

(defun cdr-nested-empty-array-execute-cb (server goal-request)
  (let* ((goal (send goal-request :goal))
         (trajectory (send goal :trajectory))
         (points (send trajectory :points))
         (positions (if points
                        (send (car points) :positions)
                      (float-vector)))
         (positions-len (length positions))
         (first-position (if (> positions-len 0) (elt positions 0) 0.0)))
    (format t "server: npts=~A positions_len=~A first=~A~%"
            (length points) positions-len first-position)
    (send server :set-succeeded
          (instance control_msgs::FollowJointTrajectory_Result :init
                    :error_code positions-len
                    :error_string (format nil "~A" first-position)))))

(setq *server*
      (instance ros::simple-action-server :init
                "cdr_empty_array_test"
                control_msgs::FollowJointTrajectory
                :execute-cb #'cdr-nested-empty-array-execute-cb))

(format t "cdr_nested_empty_array server ready~%")
(while (ros::ok)
 (send *server* :worker)
 (unix::usleep 2000))
