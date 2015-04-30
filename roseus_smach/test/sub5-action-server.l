;; sub5-action-server.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "roseus_smach")

(ros::roseus "sub5_action_server")

(defun sub5-callback (server goal)
  (let* ((input (send goal :goal :value))
        (output input))
    (dotimes (i 5)
      (unix:sleep 1)
      (setq output (- output 1))
      (setq fmsg (send server :feedback :value output))
      (send server :publish-feedback fmsg))
    (setq msg (send server :result :value output))
    (send server :set-succeeded msg)))

(setq s (instance ros::simple-action-server :init
                  "sub5" roseus_smach::Sub5Action
                  :execute-cb 'sub5-callback))

(ros::rate 10)
(do-until-key
 (send s :worker)
  (ros::spin-once)
  (ros::sleep))

(exit)
