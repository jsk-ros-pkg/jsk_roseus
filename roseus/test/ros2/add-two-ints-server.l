(ros::roseus-add-srvs "example_interfaces")
(ros::roseus "add_two_ints_server")
(ros::advertise-service "add_two_ints" example_interfaces::AddTwoInts
                        #'(lambda (req)
                            (let ((res (send req :response)))
                              (send res :sum (+ (send req :a) (send req :b)))
                              res)))
(ros::create-timer 30.0 #'(lambda (event) (exit 0)) :oneshot t)
(ros::spin)
