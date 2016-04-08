;; publish-tabletop-scenario.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "publish_tabletop_scenario")
(ros::rate 10)

(load "models/karimoku-1200-desk-object.l")
(load "models/petbottle-object.l")
(load "models/mac-burger-box-object.l")

(defparameter *cloud-topic* "/cloud_out")
(ros::advertise *cloud-topic* sensor_msgs::PointCloud2 10)
(setq *tfb* (instance ros::transform-broadcaster :init))

(setq *camera-model*
      (make-camera-from-param :pwidth 640 :pheight 360
                              :fx 400 :fy 400
                              :cx 319.5 :cy 179.5 :name "camtest"
                              :create-viewer nil
                              ))

(send *camera-model* :translate #f(0 0 1300) :world)
(send *camera-model* :rotate (deg2rad 140) :y)
(send *camera-model* :rotate -pi/2 :z)

(setq *table* (karimoku-1200-desk))
(send *table* :translate #f(700 0 0))

(setq *obj1* (petbottle))
(send *obj1* :translate #f(500 0 700))

(setq *obj2* (mac-burger-box))
(send *obj2* :translate #f(450 -150 730))

(send *camera-model* :draw-objects (list *table* *obj1* *obj2*))
(ros::sleep)

(setq *ret* (send *camera-model* :get-image :with-points t :with-colors t))
(setq *pc* (cdr *ret*))

(setq *pc-msg* (make-ros-msg-from-eus-pointcloud *pc* :frame "camera_link"))


(while (ros::ok)
  (send *pc-msg* :header :stamp (ros::time-now))
  (ros::publish *cloud-topic* *pc-msg*)
  (send *tfb* :send-transform (send *camera-model* :worldcoords) "/world" "/camera_link")
  (send *tfb* :send-transform (send *obj1* :worldcoords) "/world" "/object1")
  (send *tfb* :send-transform (send *obj2* :worldcoords) "/world" "/object2")
  (ros::spin-once)
  (ros::sleep))
(exit)

(provide :publish-tabletop-scenario) ;; end of publish-tabletop-scenario.l
