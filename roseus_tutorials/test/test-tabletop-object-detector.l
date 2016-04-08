(ros::roseus "test_tabletop_object_detector")
(ros::rate 10)

(ros::load-ros-manifest "posedetection_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "visualization_msgs")

(defvar *result-topic* "/tabletop/ObjectDetection")
(defvar *visualize-topic* "/tabletop_detection_marker_array")
(defvar *bounding-box-array-topic* "/tabletop_object_bounding_box_array")

(require :unittest "lib/llib/unittest.l")

(init-unit-test)

(setq *result-msg* nil)
(setq *visualize-msg* nil)
(setq *bbox-msg* nil)

(ros::subscribe *result-topic* posedetection_msgs::ObjectDetection
    #'(lambda (m) (setq *result-msg* m)))
(ros::subscribe *visualize-topic* visualization_msgs::MarkerArray
    #'(lambda (m) (setq *visualize-msg* m)))
(ros::subscribe *bounding-box-array-topic* jsk_recognition_msgs::BoundingBoxArray
    #'(lambda (m) (setq *bbox-msg* m)))

(setq *tfl* (instance ros::transform-listener :init))

(deftest test-published-msg ()
  (setq start (ros::time-now))
  (while (and
          (< (send (ros::time- (ros::time-now) start) :to-sec) 10)
          (null (and *result-msg* *visualize-msg* *bbox-msg*)))
    (ros::ros-info "waiting messages ~A sec" (send (ros::time- (ros::time-now) start) :to-sec))
    (ros::spin-once)
    (ros::sleep))

  (assert *result-msg* "~A is not published" *result-topic*)
  (assert *visualize-msg* "~A is not published" *visualize-topic*)
  (assert *bbox-msg* "~A is not published" *bounding-box-array-topic*)

  (setq table->obj1 (send *tfl* :lookup-transform "/table" "/object1" (ros::time 0)))
  (ros::ros-info "table->obj1 ~A" table->obj1)
  (setq table->obj2 (send *tfl* :lookup-transform "/table" "/object2" (ros::time 0)))
  (ros::ros-info "table->obj2 ~A" table->obj2)

  (setq objects (mapcar #'(lambda (o)
                            (ros::tf-pose->coords (send o :pose)))
                        (send *result-msg* :objects)))
  (setq diff1 (norm (send (elt objects 0) :difference-position table->obj1)))
  (assert (< diff1 150)
          "invalid object1 position (diff: ~A)" diff1)
  (setq diff2 (norm (send (elt objects 1) :difference-position table->obj2)))
  (assert (< diff2 150)
          "invalid object2 position (diff: ~A)" diff2))

(run-all-tests)
(exit)

