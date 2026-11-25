#!/usr/bin/env roseus
;;;
;;; Regression test client for CDR serialization of nested messages that
;;; contain EMPTY dynamic arrays (see test-cdr-nested-empty-array-server.l).
;;;
;;; Sends FollowJointTrajectory goals whose trajectory point has a non-empty
;;; `positions` array but empty `velocities`/`accelerations`/`effort`. Before
;;; the rosidl_generator_eus alignment fix, :send-goal failed to serialize such
;;; a goal ("Fast CDR exception deserializing ... FollowJointTrajectory_SendGoal
;;; _Request") and the goal never reached the server, so :wait-for-result timed
;;; out. This test exits non-zero on any such failure.
;;;

(ros::load-ros-package "control_msgs")
(require :actionlib2 "actionlib2.l")

(ros::roseus "cdr_nested_empty_array_client")

(defun make-name-list (n prefix)
  (let (ret)
    (dotimes (i n (nreverse ret))
      (push (format nil "~A_~A" prefix i) ret))))

(defun make-goal (index n)
  (let* ((names (make-name-list n (format nil "joint_~A" index)))
         (positions (instantiate float-vector n))
         (point (instance trajectory_msgs::JointTrajectoryPoint :init))
         (trajectory
          (instance trajectory_msgs::JointTrajectory :init :joint_names names))
         (goal (instance control_msgs::FollowJointTrajectory_Goal :init)))
    (dotimes (i n)
      (setf (elt positions i) (+ (* index 1.0) (* i 0.25))))
    ;; positions is non-empty; velocities/accelerations/effort stay EMPTY
    (send point :positions positions)
    (send point :time_from_start :sec 1)
    (send trajectory :points (list point))
    (send trajectory :header :frame_id (format nil "frame_~A_len_~A" index n))
    (send goal :trajectory trajectory)
    goal))

(defun send-and-check (client index n)
  (let* ((goal (make-goal index n))
         (expected-first (if (> n 0) (* index 1.0) 0.0)))
    (format t "client: send index=~A positions_len=~A~%" index n)
    (send client :send-goal goal)
    (unless (send client :wait-for-result :timeout 10)
      (format *error-output*
              "FAIL: timeout (goal not delivered) index=~A positions_len=~A~%"
              index n)
      (exit 1))
    (let ((result (send client :get-result)))
      (unless result
        (format *error-output* "FAIL: nil result index=~A~%" index)
        (exit 1))
      (let ((positions-len (send result :error_code))
            (first-position (read-from-string (send result :error_string))))
        (format t "client: decoded positions_len=~A first=~A~%"
                positions-len first-position)
        (unless (= positions-len n)
          (format *error-output*
                  "FAIL: positions length mismatch got ~A expected ~A~%"
                  positions-len n)
          (exit 1))
        (unless (eps= first-position expected-first)
          (format *error-output*
                  "FAIL: first position mismatch got ~A expected ~A~%"
                  first-position expected-first)
          (exit 1))))))

(setq *client*
      (instance ros::simple-action-client :init
                "cdr_empty_array_test"
                control_msgs::FollowJointTrajectory))
(unless (send *client* :wait-for-server 10)
  (format *error-output* "FAIL: cdr_empty_array_test server is not available~%")
  (exit 1))

;; exercise a range of positions lengths so the empty arrays land at several
;; alignment offsets (this is what triggered the original corruption)
(let ((sizes '(0 1 2 3 5 8 13 21 29)))
  (dotimes (i (length sizes))
    (send-and-check *client* i (elt sizes i))))
(format t "client: all goals round-tripped successfully~%")
(exit 0)
