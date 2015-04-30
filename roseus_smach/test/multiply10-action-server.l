;; multiply10-action-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "roseus_smach")

(ros::roseus "multiply10_action_server")

(defun multiply10-callback (server goal)
  (let* ((input (send goal :goal :value))
         (output 0))
    (dotimes (i 10)
      (unix:sleep 1)
      (setq output (+ output input))
      (setq fmsg (send server :feedback :value output))
      (send server :publish-feedback fmsg))
    (setq msg (send server :result :value output))
    (send server :set-succeeded msg)))

(setq s (instance ros::simple-action-server :init
                  "multiply10" roseus_smach::Multiply10Action
                  :execute-cb 'multiply10-callback))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)

