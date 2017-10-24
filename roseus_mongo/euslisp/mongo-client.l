;; mongo-client.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "roseus_mongo_client")

(ros::load-ros-manifest "mongodb_store_msgs")

(require :json-encode "package://roseus_mongo/euslisp/json/json-encode.l")
(require :json-decode "package://roseus_mongo/euslisp/json/json-decode.l")

(unless (find-package "MONGO") (make-package "MONGO"))
(in-package "MONGO")

(defvar *mongo-database* (ros::get-param "robot/database" "test"))
(defvar *mongo-collection* (ros::get-param "robot/name" "test"))
(defvar *mongo-query-default-buffer-size* 4096)
(defvar *mongo-service-query* "/message_store/query_messages")
(defvar *mongo-service-insert* "/message_store/insert")
(defvar *mongo-service-update* "/message_store/update")
(defvar *mongo-service-delete* "/message_store/delete")
(defvar *mongo-service-timeout* (floor (ros::get-param "~mongodb_timeout" 1)))

(defun wait-for-server (&optional (timeout 30))
  "Wait for mongodb server to be ready"
  (when (ros::get-param "/mongodb_use_daemon" nil)
    (if (ros::wait-for-service *mongo-service-query* timeout)
        (return-from wait-for-server t))
    (ros::ros-warn "timedout. mongodb daemon server is not ready.")
    (return-from wait-for-server nil))
  ;; without daemon
  (unless (ros::wait-for-service "/datacentre/wait_ready" timeout)
    (ros::ros-warn "timeout. mongodb server is not ready.")
    (return-from wait-for-server nil))
  (user::call-empty-service "/datacentre/wait_ready")
  (unix:sleep 2) ;; omajinai
  t)

(defun shutdown-server (&optional (timeout 10))
  "Request shutdown to mongodb server"
  (unless (ros::get-param "/mongodb_user_daemon" nil)
    (ros::ros-warn "mongodb server is running on daemon mode. Please shutdown server using system command.")
    (return-from shutdown-server t))
  (unless (ros::wait-for-service "/datacentre/shutdown" timeout)
    (ros::ros-error "timeout. Something goes wrong with mongodb server.")
    (return-from shutdown-server nil))
  (user::call-empty-service "/datacentre/shutdown")
  (ros::ros-info "send shutdown signal to mongodb server")
  t)

(defun apply-to-ros-query (encoder lst)
  (let ((ss (make-string-output-stream *mongo-query-default-buffer-size*))
        (msg (instance mongodb_store_msgs::StringPairList :init)))
    (when lst
      (funcall encoder lst ss)
      (ros::ros-debug "query: ~A~%" (get-output-stream-string ss))
      (send msg :pairs
            (list
             (instance mongodb_store_msgs::StringPair :init
                       :first mongodb_store_msgs::MongoQueryMsgRequest::*JSON_QUERY*
                       :second (get-output-stream-string ss)))))
    msg))

(defun alist->ros-query (alist)
  (apply-to-ros-query #'json::encode-alist alist))

(defun plist->ros-query (plist)
  (apply-to-ros-query #'json::encode-plist plist))

(defun query (msg-type &key query meta sort
                            single limit (encoder #'json::encode-alist) msg-only)
  "Find message from server. See `mongo-client-sample.l` for example.
   `(mongo::query geometry_msgs::Pose)` returns all messages of type `geometry_msgs/Pose` on database with their meta fields:
   `((#<geometry_msgs::pose #X6ec9630> (:_id (:$oid . \"59edc3443c607c77a45710c8\")) (:stored_type . \"geometry_msgs/Pose\") (:stored_class . \"geometry_msgs.msg._Pose.Pose\") (:inserted_by . \"/roseus_mongo_client_1508754226103673004\") (:inserted_at . #<calendar-time #X6ec9ef8 2017-10-23 19:24:04>))
    ... )`

   Args:
      msg-type: A class of message to find
                e.g. `geometry_msgs::Pose`
      query: Condition for searching message on database
             e.g. To find messages of type `geometry_msgs/Pose` class which have `pose.position.x == 2`, run:
                    `(mongo::query geometry_msgs::Pose :query '((:position.x . 2)))`
                  Also you can add conditions like `AND` search:
                    `(mongo::query geometry_msgs::Pose :query '((:position.x . 2) (:orientation.w . 1)))`
      meta: Condition for meta field of documents on database.
            Each messages have meta fields. This argument is to find by meta fields.
            e.g. To find message of `std_msgs/String` inserted by `/talker` node:
                 `(mongo::query std_msgs::String :meta '((:inserted_by . \"/talker\")))`
      sort: Condifition for sorting results
            e.g. To get the TOP 5 reliable result messages of type `posedetection_msgs/Object6DPose`:
                 `(mongo::query posedetection_msgs::Object6DPose
                                :sort '((:reliability . -1))
                                :limit 5)`
      single: If enabled, gets only the first matched result on database.
      limit: Limits the number of result messages.
      msg-only: If enabled, returns only messages without any meta fields."

  (let ((req (instance mongodb_store_msgs::MongoQueryMsgRequest :init))
        res)
    (send req :database *mongo-database*)
    (send req :collection *mongo-collection*)
    (send req :type (get msg-type :datatype-))
    (send req :single single)
    (when limit
      (if (memq :limit (send req :methods))
           (send req :limit limit) ;; valid on ROS indigo and later
        (ros::ros-warn "feature :limit is provided only indigo and later")))
    (when query
      (send req :message_query (apply-to-ros-query encoder query)))
    (when meta
      (send req :meta_query (apply-to-ros-query encoder meta)))
    (when sort
      (send req :sort_query (apply-to-ros-query encoder sort)))
    (unless (ros::wait-for-service *mongo-service-query* *mongo-service-timeout*)
      (ros::ros-warn "wait-for-service ~A timeout." *mongo-service-query*)
      (return-from query nil))
    (setq res (ros::service-call *mongo-service-query* req))
    (mapcar #'(lambda (message meta-pair)
                (let ((msg (instance msg-type :init))
                      (meta-raw-string (send (car (send meta-pair :pairs)) :second)))
                  (send msg :deserialize (send message :msg))
                  (if msg-only msg
                      (cons msg (json::parse-from-string meta-raw-string)))))
            (send res :messages) (send res :metas))))

(defun query-by-id (msg-type id &key msg-only)
  "Find message by message id from server."
  (car (query msg-type :query `((:_id (:$oid . ,id))) :single t :msg-only msg-only)))

(defun insert (msg &key meta (encoder #'json::encode-alist) (wait t))
  "Inserts message into mongo database.

   Returns:
     A document id of inserted data if `:wait` is enabled (by default), otherwise returns T.

   Arguments:
     msg: message to be inserted
     meta: additional meta fields
           e.g. `(mongo::insert msg :meta '((:memo . \"my awesome pose\")))`
     wait: waits for server to insert data if enabled."
  (let ((data (instance mongodb_store_msgs::SerialisedMessage :init
                        :type (send msg :datatype-)
                        :msg (send msg :serialize)))
        (meta-data (apply-to-ros-query encoder meta)))
    (unless wait
      (unless (ros::get-topic-publisher *mongo-service-insert*)
        (ros::advertise *mongo-service-insert* mongodb_store_msgs::Insert 10)
        (unix:sleep 1))
      (ros::publish
       *mongo-service-insert*
       (instance mongodb_store_msgs::Insert :init
                 :database *mongo-database*
                 :collection *mongo-collection*
                 :message data
                 :meta meta-data))
      (return-from insert t))
    ;; :wait == t
    (let ((req (instance mongodb_store_msgs::MongoInsertMsgRequest :init
                         :database *mongo-database*
                         :collection *mongo-collection*
                         :message data
                         :meta meta-data))
          res)
    (unless (ros::wait-for-service *mongo-service-insert* *mongo-service-timeout*)
      (ros::ros-warn "wait-for-service ~A timeout." *mongo-service-insert*)
      (return-from insert nil))
    (setq res (ros::service-call *mongo-service-insert* req))
    (send res :id))))

(defun update (msg &key meta query meta-query upsert (encoder #'json::encode-alist))
  "Updates a message.

   Returns:
     A ID of updated message.

   Arguments:
     msg: message to be inserted.
     meta: additional meta fields. If empty, leaves meta unchanged. If empty on upsert is true, uses meta-query.
     query: conditions on message content to be updated.
     meta-query: conditions on meta content to be updated.
     upsert: do `insert` if no matching message found."
  (let ((req (instance mongodb_store_msgs::MongoUpdateMsgRequest :init))
        res)
    (send req :database *mongo-database*)
    (send req :collection *mongo-collection*)

    (when query
      (send req :message_query (apply-to-ros-query encoder query)))
    (when meta-query
      (send req :meta_query (apply-to-ros-query encoder meta-query)))
    (send req :upsert upsert)

    (send req :message (instance mongodb_store_msgs::SerialisedMessage :init
                                 :type (send msg :datatype-)
                                 :msg (send msg :serialize)))
    (when meta
      (send req :meta (apply-to-ros-query encoder meta)))

    (unless (ros::wait-for-service *mongo-service-update* *mongo-service-timeout*)
      (ros::ros-error "wait-for-service ~A timedout." *mongo-service-update*)
      (return-from update nil))
    (setq res (ros::service-call *mongo-service-update* req))
    (send res :id)))

(defun update-by-id (id msg &key meta upsert (encoder #'json::encode-alist))
  "Updates a message by object ID."
  (update msg :query `((:_id (:$oid . ,id))) :meta meta :upsert upsert))

(defun delete-by-id (id)
  "Deletes data from server"
  (let ((req (instance mongodb_store_msgs::MongoDeleteMsgRequest :init))
        res)
    (send req :database *mongo-database*)
    (send req :collection *mongo-collection*)
    (send req :document_id id)
    (unless
        (ros::wait-for-service *mongo-service-delete* *mongo-service-timeout*)
      (ros::ros-warn "wait-for-service ~A timeout." *mongo-service-delete*)
      (return-from delete-by-id nil))
    (setq res (ros::service-call *mongo-service-delete* req))
    (send res :success)))

(defun replicate (&key
                    (database *mongo-database*)
                    (collections (list *mongo-collection*))
                    (move-before (ros::time (* 60 60 24)))
                    (delete-after-move nil))
  (let ((c (instance ros::simple-action-client :init
                     "/move_mongodb_entries" mongodb_store_msgs::MoveEntriesAction :groupname "move_entries"))
        (goal (instance mongodb_store_msgs::MoveEntriesActionGoal :init
                        :header (instance std_msgs::Header :init :stamp (ros::time-now))
                        :goal
                        (instance mongodb_store_msgs::MoveEntriesGoal :init
                        :database database
                        :collections (instance mongodb_store_msgs::StringList :init
                                               :data collections)
                        :move_before move-before
                        :delete_after_move delete-after-move))))
    (unless (send c :wait-for-server *mongo-service-timeout*)
      (ros::ros-warn "wait-for-server /move_mongodb_entries timeout")
      (return-from replicate nil))
    (send c :send-goal-and-wait goal))
  t)

(provide :mongo-client) ;; end of mongo-client.l
