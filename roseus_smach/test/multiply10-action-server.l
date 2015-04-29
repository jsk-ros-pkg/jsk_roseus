;; multiply10-action-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "roseus_smach")

(ros::roseus "multiply10_action_server")

(defun multiply10-callback (server goal)
  (let ((input (send goal :goal :value)))
    (unix:sleep 10)
    (setq msg (send server :result :value (* 10 input)))
    (send server :set-succeeded msg)))

(setq s (instance ros::simple-action-server :init
                  "multiply10" roseus_smach::Multiply10Action
                  :execute-cb #'multiply10-callback))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

