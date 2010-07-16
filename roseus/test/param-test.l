#!/usr/bin/env runeus

(unless (find "roseus/euslisp/" *load-path* :test #'substringp)
  (pushnew (format nil "~A/euslisp/"
                   (read-line (piped-fork "rospack find roseus")))
           *load-path* :test #'equal))
(load "roseus.l")
(print 'hgoe)
(roseus "param_test")
(unix:sleep 1)
(ros::ros-info "~~test => ~A" (ros::get-param "~test"))
(ros::ros-info "test => ~A" (ros::get-param "test"))
(ros::ros-info "/test => ~A" (ros::get-param "/test"))
(ros::spin)
