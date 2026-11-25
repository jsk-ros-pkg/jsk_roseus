#!/usr/bin/env roseus

(ros::load-ros-package "control_msgs")
(require :actionlib2 "actionlib2.l")

(ros::roseus "follow_joint_trajectory_server_test")

(defun make-name-list (n prefix)
  (let (ret)
    (dotimes (i n (nreverse ret))
      (push (format nil "~A_~A" prefix i) ret))))

(defun follow-joint-trajectory-execute-cb (server goal-request)
  (let* ((goal (send goal-request :goal))
         (trajectory (send goal :trajectory))
         (joint-names (send trajectory :joint_names))
         (n (length joint-names)))
    (format t "server: goal joint_names_len=~A~%" n)
    (send server :set-succeeded
          (instance control_msgs::FollowJointTrajectory_Result :init
                    :error_code 0
                    :error_string
                    (format nil "ok joint_names_len=~A first=~A"
                            n
                            (if (> n 0) (car joint-names) ""))))))

(setq *server*
      (instance ros::simple-action-server :init
                "follow_joint_trajectory_test"
                control_msgs::FollowJointTrajectory
                :execute-cb #'follow-joint-trajectory-execute-cb))

(format t "follow_joint_trajectory_repro server ready~%")
(while (ros::ok)
 (send *server* :worker)
 (unix::usleep 2000))
