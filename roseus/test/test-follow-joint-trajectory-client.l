#!/usr/bin/env roseus

(ros::load-ros-package "control_msgs")
(require :actionlib2 "actionlib2.l")

(ros::roseus "follow_joint_trajectory_client_test")

(defun script-args (script-name)
  (let ((pos (position script-name lisp::*eustop-argument*
                       :test #'substringp)))
    (if pos
        (nthcdr (1+ pos) lisp::*eustop-argument*)
      nil)))

(defun make-name-list (n prefix)
  (let (ret)
    (dotimes (i n (nreverse ret))
      (push (format nil "~A_~A" prefix i) ret))))

(defun make-goal (index n)
  (let* ((trajectory
          (instance trajectory_msgs::JointTrajectory :init
                    :joint_names
                    (make-name-list n (format nil "goal_~A_joint" index))))
         (goal
          (instance control_msgs::FollowJointTrajectory_Goal :init
                    :trajectory trajectory)))
    (send trajectory :header :frame_id
          (format nil "goal_frame_~A_len_~A" index n))
    goal))

(defun send-variable-goal (client index n)
  (let ((goal (make-goal index n)))
    (format t "client: send index=~A joint_names_len=~A~%" index n)
    (send client :send-goal goal)
    (unless (send client :wait-for-result :timeout 10)
      (format *error-output*
              "timeout: index=~A joint_names_len=~A~%" index n)
      (exit 1))
    (let ((result (send client :get-result)))
      (unless result
        (format *error-output*
                "nil result: index=~A joint_names_len=~A~%" index n)
        (exit 1))
      (format t "client: result index=~A error_code=~A error_string_len=~A error_string=~A~%"
              index
              (send result :error_code)
              (length (send result :error_string))
              (send result :error_string)))))

(let* ((args (script-args "test-follow-joint-trajectory-client.l"))
       (loops (if args (read-from-string (car args)) 200))
       (sizes (if (cdr args)
                  (read-from-string (cadr args))
                '(0 1 2 3 5 8 13 21 34 55 89 144 233 377 610))))
  (setq *client*
        (instance ros::simple-action-client :init
                  "follow_joint_trajectory_test"
                  control_msgs::FollowJointTrajectory))
  (unless (send *client* :wait-for-server 10)
    (format *error-output*
            "follow_joint_trajectory_repro server is not available~%")
    (exit 1))
  (dotimes (i loops)
    (send-variable-goal *client* i (elt sizes (mod i (length sizes)))))
  (format t "client: completed ~A goals~%" loops)
  (exit 0))
