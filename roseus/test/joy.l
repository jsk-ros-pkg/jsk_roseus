#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")

(defclass joy
  :super propertied-object
  :slots (axes buttons axis-mapping button-mapping))

(defmethod joy
  (:init
   ()
   (setq axes (instantiate float-vector
                           (1+ (cadr (find-extream axis-mapping #'cadr #'>))))
         buttons (instantiate float-vector
                              (1+ (cadr (find-extream button-mapping #'cadr #'>)))))
   (dolist (mapping button-mapping)
     (eval `(defmethod joy (,(car mapping) ()
                            (elt buttons ,(cadr mapping))))))
   (dolist (mapping axis-mapping)
     (eval `(defmethod joy (,(car mapping) ()
                            (elt axes ,(cadr mapping))))))
   self)
  (:button-names () (mapcar #'car button-mapping))
  (:axis-names () (mapcar #'car axis-mapping))
  (:axes () axes)
  (:buttons () buttons)
  (:joy-cb
   (msg)
   (setq axes (send msg :axes)
         buttons (send msg :buttons))
   self)
  )

(defclass ps3-joy
  :super joy
  :slots ())
(defmethod ps3-joy
  (:init
   ()
   (setq axis-mapping
         '((:ljoy-lr 0)
           (:ljoy-fb 1)
           (:rjoy-lr 2)
           (:rjoy-fb 3)
           (:arrow-f 4)
           (:arrow-b 6)
           (:arrow-r 5)
           (:arrow-l 7)
           (:acceleration-sensor-rl 16)
           (:acceleration-sensor-fb 17)
           (:acceleration-sensor-ud 18)
           (:gyro-sensor-yaw 19)
           )
         button-mapping
         '((:select-button 0)
           (:l3-button 1)
           (:r3-button 2)
           (:start-button 3)
           (:up-button 4)
           (:right-button 5)
           (:down-button 6)
           (:left-button 7)
           (:l2-button 8)
           (:r2-button 9)
           (:l1-button 10)
           (:r1-button 11)
           (:triangle-button 12)
           (:circle-button 13)
           (:cross-button 14)
           (:square-button 15)
           (:pairing-button 16)
           ))
   (send-super :init))
  )

(defclass wii-joy
  :super joy
  :slots ())
(defmethod wii-joy
  (:init
   ()
   (setq axis-mapping
         '((:roll 0)
           (:pitch 1)
           (:yaw 2)
           )
         button-mapping
         '((:1-button 0)
           (:2-button 1)
           (:a-button 2)
           (:b-button 3)
           (:plus-button 4)
           (:minus-button 5)
           (:left-button 6)
           (:rigut-button 7)
           (:up-button 8)
           (:down-button 9)
           (:home-button 10)))
   (send-super :init))
  )

(defun init-ps3joy ()
  (setq *joy* (instance ps3-joy :init))
  (ros::roseus "joy")
  (ros::subscribe "/joy" sensor_msgs::Joy  #'send *joy* :joy-cb))

(defun init-wiijoy ()
  (setq *joy* (instance wii-joy :init))
  (ros::roseus "joy")
  (ros::subscribe "/joy" sensor_msgs::Joy  #'send *joy* :joy-cb))

(defun sample-ps3joy ()
  (ros::rate 10)
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   (pprint
    (append
     (mapcar #'(lambda (meth) (list meth (send *joy* meth))) (send *joy* :button-names))
     (mapcar #'(lambda (meth) (list meth (send *joy* meth))) (send *joy* :axis-names))))
   )
  )

;; how to run sample
;; 1. prepare ps3joy according to http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle
;; 2. rosrun joy joy_node
;; 3. rosrun roseus roseus `rospack find roseus`/test/joy.l "(progn (init-ps3joy) (sample-ps3joy))"

