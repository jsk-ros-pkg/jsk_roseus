#!/usr/bin/env roseus

(ros::load-ros-manifest "wiimote")

(defun ros::vector3->vector (vec)
  (float-vector (send vec :x) (send vec :y) (send vec :z)))
(defun ros::quaternion->matrix (quaternion)
  (user::quaternion2matrix
   (float-vector (send quaternion :w)
                 (send quaternion :x)
                 (send quaternion :y) 
                 (send quaternion :z))))

(setq *imu-orientation* (unit-matrix 3)
      *imu-angularvelocity* (float-vector 0 0 0)
      *imu-linearacceleration* (float-vector 0 0 0)
      *joy-axes* (float-vector 0 0 0)
      *joy-buttons* (integer-vector 0 0 0 0 0 0 0 0 0 0 0)
      *wiimote-buttons* (make-array 11)
      *wiimote-leds* (make-array 4)
      *wiimote-irtracking* nil
      *wiimote-rawbattery* 0.0
      *wiimote-percentbattery* 0.0
      )

(ros::roseus "wiimote")
(ros::subscribe "/imu/data" sensor_msgs::Imu
		#'(lambda (msg)
		    (setq *imu-orientation*
			  (ros::quaternion->matrix (send msg :orientation))
			  *imu-angularvelocity*
			  (ros::vector3->vector (send msg :angular_velocity))
			  *imu-linearacceleration*
			  (ros::vector3->vector (send msg :linear_acceleration)))))

(ros::subscribe "/joy" sensor_msgs::Joy
		#'(lambda (msg)
		    (setq *joy-axes* (send msg :axes)
			  *joy-buttons* (send msg :buttons))))

(ros::subscribe "/wiimote/state" wiimote::State
		#'(lambda (msg)
		    (setq *wiimote-buttons* (send msg :buttons)
			  *wiimote-leds* (send msg :leds)
			  *wiimote-irtracking*
			  (if (> (send (elt (send msg :ir_tracking) 0) :ir_size) 0)
			      (float-vector
			       (send (elt (send msg :ir_tracking) 0) :x)
			       (send (elt (send msg :ir_tracking) 0) :y))
			    nil)
			  *wiimote-rawbattery* (send msg :raw_battery)
			  *wiimote-percentbattery* (send msg :percent_battery)
			  )))

(ros::advertise "/wiimote/rumble" wiimote::RumbleControl 5)
(ros::advertise "/wiimote/leds" wiimote::LEDControl 5)

(ros::spin-once) ;; spin once before publish

(ros::rate 10)
(dotimes (i 2)
  (ros::publish "/wiimote/rumble"
		(instance wiimote::RumbleControl :init
			  :rumble
			  (instance wiimote::TimedSwitch :init
				    :switch_mode -1
				    :num_cycles  2
				    :pulse_pattern (float-vector 0.2 0.1 0.6 1.0)))))

;; led
(setq led-on (instance wiimote::TimedSwitch :init :switch_mode 1 :num_cycles -1))
(setq led-off (instance wiimote::TimedSwitch :init :switch_mode 0 :num_cycles -1))

;; main
(setq i 0)
(do-until-key
  (format t "imu   orientation         ~A~%" *imu-orientation*)
  (format t "      angular-velocity    ~A~%" *imu-angularvelocity*)
  (format t "      linear-acceleration ~A~%" *imu-linearacceleration*)
  (format t "joy   axes                ~A~%" *joy-axes*)
  (format t "      buttons             ~A~%" *joy-buttons*)
  (format t "state buttons             ~A~%" *wiimote-buttons*)
  (format t "      leds                ~A~%" *wiimote-leds*)
  (format t "      ir-tracking         ~A~%" *wiimote-irtracking*)
  (format t "      raw-battery         ~A~%" *wiimote-rawbattery*)
  (format t "      percent-battery     ~A~%" *wiimote-percentbattery*)
  (ros::publish
   "/wiimote/leds"
   (instance wiimote::LEDControl :init
	     :timed_switch_array (list (if (= (mod i 4) 0) led-on led-off)
				       (if (= (mod i 4) 1) led-on led-off)
				       (if (= (mod i 4) 2) led-on led-off)
				       (if (= (mod i 4) 3) led-on led-off))))
  (print (mod i 4))
  (ros::spin-once)
  (ros::sleep)
  (incf i)
  )
;;
(ros::publish
 "/wiimote/leds"
 (instance wiimote::LEDControl :init
	   :timed_switch_array (list led-off led-off led-off led-off)))
(ros::spin-once)
(ros::sleep)

