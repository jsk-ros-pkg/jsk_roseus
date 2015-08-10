;; mongo-client-sample.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")

;; find all messages of type geometry_msgs/Pose
(mongo::query geometry_msgs::Pose)

;; find one message of all geometry_msgs/Pose
(mongo::query geometry_msgs::Pose :single t)

;; find all geometry_msgs/Pose messages whose position.x = 2
(mongo::query geometry_msgs::Pose
              :query '((:position.x . 2)))

;; find all geometry_msgs/Pose sorted by dates newest to oldest
(mongo::query geometry_msgs::Pose
              :sort '((:natural . -1)))

;; find all geometry_msgs/Pose which were inserted after 2015/7/17
(mongo::query geometry_msgs::Pose
              :meta '((:inserted_at .
                       ((:$gt . ((send (instance calendar-time :init)
                                  :make :year 2015 :month 7 :day 17)))))))

;; inserting message to database
(setq msg (instance geometry_msgs::Pose :init))
(setq document-id (mongo::insert msg))

;; deleting message from database
(setq success-p (mongo::delete document-id))

(provide :mongo-client-sample) ;; end of mongo-client-sample.l
