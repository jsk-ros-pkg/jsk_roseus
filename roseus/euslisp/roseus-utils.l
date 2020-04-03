;; this is a collection of utility functions and macros
;; for ros<->euslisp interface

;; load c-binding library
(load-library "roseus_c_util.so" '("roseus_c_util"))
(ros::roseus-add-srvs "dynamic_reconfigure")
(ros::roseus-add-msgs "dynamic_reconfigure")

(defun print-ros-msg (msg &optional (rp (find-package "ROS"))
                          (ro ros::object) (nest 0) (padding "  "))
  (dolist (s (let ((slt (send msg :slots)))
               ;; `s' is a list of slots unique to the msg object
               ;; i.e. slots present in `ro' and inherited by `msg' are filtered
               (map nil #'(lambda (obj) (setq slt (remove obj slt :key #'car :count 1)))
                    (send ro :slots))
               slt))
    (let ((sm (car s)))
      (when (and (symbolp sm) (eq (symbol-package sm) rp))
        (cond
         ((derivedp (cdr s) ro)
          (dotimes (i nest) (format t padding))
          (format t ":~A -- ~A~%"
                  (string-downcase (string-left-trim "_" (symbol-string sm)))
                  (send (class (cdr s)) :name))
          (print-ros-msg (cdr s) rp ro (1+ nest) padding))
         ((and (listp (cdr s))
               (derivedp (cadr s) ro))
          (dotimes (i nest) (format t padding))
          (format t ":~A [ ~A ]~%"
                  (string-downcase (string-left-trim "_" (symbol-string sm)))
                  (send (class (cadr s)) :name))
          (dotimes (i (1+ nest)) (format t padding)) (format t "[~%")
          (dolist (ss (cdr s))
            (print-ros-msg ss rp ro (+ nest 2) padding))
          (dotimes (i (1+ nest)) (format t padding)) (format t "]~%")
          )
         (t
          (dotimes (i nest) (format t padding))
          (format t ":~A~%" (string-downcase (string-left-trim "_" (symbol-string sm))))
          (dotimes (i nest) (format t padding))
          (print (cdr s)))
         )))))

;; Sensors
(ros::roseus-add-msgs "sensor_msgs")

(defun make-camera-from-ros-camera-info-aux (pwidth pheight p frame-coords &rest args)
  (let* ((fx (elt p 0))(fy (elt p 5))
         (cx (elt p 2))(cy (elt p 6))
         (fx*tx (elt p 3))(fy*ty (elt p 7))
         (tx (* 1000 (/ fx*tx fx)))
         (ty (* 1000 (/ fy*ty fy)))
         )
    (apply #'make-camera-from-param :pwidth pwidth :pheight pheight
	   :fx fx :fy fy :cx cx :cy cy
	   :tx tx :ty ty :parent-coords frame-coords args)))

(defun make-camera-from-ros-camera-info (msg)
  "Make camera model from ros sensor_msgs::CameraInfo message"
  (let* ((pwidth (send msg :width))
         (pheight (send msg :height))
         (frame-id (send msg :header :frame_id))
         (p (send msg :P))
         c parent)
    (if (and (> (length frame-id) 0) (eq (elt frame-id 0) #\/))
        (setq frame-id (subseq frame-id 1)))
    (if (not (boundp '*l*))
        (setq *l* (instance ros::transform-listener :init)))
    (setq parent (send *l* :get-parent frame-id (ros::time)))
    (unless parent (return-from make-camera-from-ros-camera-info nil))
    (setq c (send *l* :lookup-transform parent frame-id (ros::time)))
    (unless c (return-from make-camera-from-ros-camera-info nil))
    (format t "(setq ~A-coords (make-coords :pos ~A :rpy ~A))~%" frame-id (send c :pos) (coerce (car (send c :rpy-angle)) float-vector))
    (format t "(send ~A-coords :transform ~A)~%" frame-id parent)
    (format t "(send ~A :assoc ~A-coords)~%" parent frame-id)
    (format t "(make-camera-from-ros-camera-info-aux ~A ~A ~A ~A-coords :name \"~A\")~%"
            pwidth pheight p frame-id frame-id)
    (make-camera-from-ros-camera-info-aux pwidth pheight p nil :name frame-id)))

;; Image
(when (and
       (assoc :create-viewer (send camera-model :methods))
       (probe-file (format nil "~A/irteus/demo/sample-camera-model.l" *eusdir*)))
(defun ros::image->sensor_msgs/Image (img &key header (seq 0) (stamp (ros::time-now)) (frame_id ""))
  (let ((img24 (swap-rgb (send (send img :copy) :to24)))
        (msg (instance sensor_msgs::Image :init)))
    (if header
        (send msg :header header)
      (send msg :header
            (instance std_msgs::header :init
                      :seq seq :stamp stamp :frame_id frame_id)))
    (send msg :height (send img24 :height))
    (send msg :width (send img24 :width))
    (send msg :step (* (send img24 :width)
                       (send img24 :components)))
    (send msg :encoding "bgr8")
    (send msg :data (send img24 :entity))
    msg))

(defun ros::sensor_msgs/Image->image (msg)
  (let ((img24 (instance color-image24 :init
                         (send msg :width)
                         (send msg :height)
                         (copy-object (send msg :data)))))
    (when (string= (send msg :encoding) "bgr8")
      (swap-rgb img24))
    img24))
) ;; end of when
;;
;; 3dpoint cloud
;;
(defun make-msg-from-3dpointcloud (points-list &key color-list (frame "/sensor_frame"))
  "Make sensor_msgs::PointCloud from list of 3d point"
  (let (points hdr msg)
    ;; make-points
    (dolist (length points-list)
      (push (instance geometry_msgs::Point32 :init) points))
    (mapcar #'(lambda (p pt)
		(send p :x (/ (elt pt 0) 1000.0))
		(send p :y (/ (elt pt 1) 1000.0))
		(send p :z (/ (elt pt 2) 1000.0)))
	    points points-list)
    ;;
    (setq hdr (instance std_msgs::header :init
               :stamp (ros::time-now) :frame_id frame))

    (setq msg (instance sensor_msgs::PointCloud :init
                        :header hdr
                        :points points))
    msg
    ))
(defun make-eus-pointcloud-from-ros-msg1 (sensor_msgs_pointcloud)
  (let* ((pts (send sensor_msgs_pointcloud :points))
         (mat (make-matrix (length pts) 3))
         (ary (array-entity mat))
         (cntr 0))
    (dolist (p pts)
      (setf (elt ary cntr) (send p :x))
      (incf cntr)
      (setf (elt ary cntr) (send p :y))
      (incf cntr)
      (setf (elt ary cntr) (send p :z))
      (incf cntr))
    (scale 1000.0 ary ary)
    (instance pointcloud :init :points mat)))

;;


;;
;; Visualization
;;
(ros::roseus-add-msgs "visualization_msgs")

;; ros helper

(defun vector->rgba (cv &optional (alpha 1.0))
  "Convert vector to std_msgs::ColorRGBA"
  (if (vectorp cv)
      (instance std_msgs::ColorRGBA :init
                :r (elt cv 0)
                :g (elt cv 1)
                :b (elt cv 2)
                :a alpha)
    (instance std_msgs::ColorRGBA :init
              :r 0 :g 0 :b 0 :a alpha)))

;; eus shape object -> visualization_msgs::Marker

(defun cylinder->marker-msg (cyl header
				 &key ((:color col) (float-vector 1.0 0 0))
				      ((:alpha a) 1.0)
                                      ((:id idx) 0) ns lifetime)
  "Convert cylinder object to visualization_msgs::Marker"
  (send cyl :worldcoords) ;; make sure to update vertices
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*CYLINDER*
                       :header header
                       :id idx))
        (cent-coords (send cyl :copy-worldcoords))
        (height (height-of-cylinder cyl))
        (radius (radius-of-cylinder cyl)))
    (send cent-coords :translate (float-vector 0 0 (/ height 2)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation
                      (float-vector (* 2 radius) (* 2 radius) height)))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun cube->marker-msg (cb header
			    &key ((:color col) (float-vector 1.0 0 0))
   			         ((:alpha a) 1.0)
                                 ((:id idx) 0) ns lifetime)
  "Convert cube object to visualization_msgs::Marker"
  (send cb :worldcoords) ;; make sure to update vertices
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*CUBE*
                       :header header
                       :id idx))
        (cent-coords (send cb :copy-worldcoords))
        (cx (x-of-cube cb))
        (cy (y-of-cube cb))
        (cz (z-of-cube cb)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation
                      (float-vector cx cy cz)))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun sphere->marker-msg (sp header
			    &key ((:color col) (float-vector 1.0 0 0))
   			         ((:alpha a) 1.0)
                                 ((:id idx) 0) ns lifetime)
  "Convert shpere object to visualization_msgs::Marker"
  (send sp :worldcoords) ;; make sure to update vertices
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*SPHERE*
                       :header header
                       :id idx))
        (cent-coords (send sp :copy-worldcoords))
        (r (radius-of-sphere sp)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation
                      (scale 2.0 (float-vector r r r))))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun line->marker-msg (li header
                            &key ((:color col) (float-vector 1 0 0))
                                 ((:alpha a) 1.0)
                                 ((:id idx) 0)
                                 ((:scale sc) 10.0) ns lifetime)
  "Convert line object to visualization_msgs::Marker"
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*LINE_STRIP*
                       :header header
                       :id idx)))
    (send msg :points (mapcar #'(lambda (l) (ros::pos->tf-point l)) li))
    (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun line-list->marker-msg (li header
                            &key ((:color col) (float-vector 1 0 0))
                                 ((:alpha a) 1.0)
                                 ((:id idx) 0)
                                 ((:scale sc) 10.0) ns lifetime)
  "Convert list of line object to visualization_msgs::Marker"
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*LINE_LIST*
                       :header header
                       :id idx)))
    (send msg :points (mapcar #'(lambda (l) (ros::pos->tf-point l)) li))
    (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun faces->marker-msg (faces header &key
                                ((:color col) (float-vector 1 0 0))
                                ((:id idx) 0) ns lifetime)
  "Convert face objects to visualization_msgs::Marker"
  ;;(send-all faces :worldcoords) ;; make sure to update vertices
  (let* ((tri-faces
          (mapcan #'(lambda (f) (geo::face-to-triangle f))
                  faces))
         (tri-vertices
          (mapcan #'(lambda (f) (cdr (send f :vertices)))
                  tri-faces))
         (points
          (mapcar #'(lambda (p)
                      (ros::pos->tf-point p))
                  tri-vertices))
         (msg (instance visualization_msgs::Marker :init
                        :type visualization_msgs::Marker::*TRIANGLE_LIST*
                        :header header
                        :id idx)))
    (send msg :pose (ros::coords->tf-pose (make-coords)))
    (send msg :scale (ros::pos->tf-translation
                      (float-vector 1000 1000 1000)))
    (send msg :color (vector->rgba col 1.0))

    (send msg :points points)
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun object->marker-msg (obj header &key
			       coords
                               ((:color col) (float-vector 1 1 1))
			       ((:alpha a) 1.0)
                               ((:id idx) 0) ns lifetime rainbow)
  "Convert euslisp body object to visualization_msgs::Marker"
  (let ((bodies (if (find-method obj :bodies) (send obj :bodies) (list obj))))
    (send-all bodies :worldcoords) ;; make sure to update vertices
    (let* ((msg (instance visualization_msgs::Marker :init
			  :type visualization_msgs::Marker::*TRIANGLE_LIST*
			  :header header
			  :id idx))
	   (body-colors
	    (mapcar
	     #'(lambda (b)
		 (cond ((null (get b :face-color)) col)
		       (t (gl::find-color (get b :face-color)))))
	     bodies))
	   (triface-vertices-list
	    (mapcar #'(lambda (fs)
			(mapcan #'(lambda (f) (cdr (send f :vertices)))
				(send fs :faces)))
		    (mapcar #'(lambda (b)
                                (if (find-method b :glvertices)
                                    (send b :glvertices :convert-to-faceset :wrt :world)
                                  (body-to-faces b))) bodies)))
	   (color-list
	    (mapcar #'(lambda (vlist c)
			(if (derivedp c gl::colormaterial) (setq c (send c :diffuse))) ;; jsk
                        (if rainbow
                            (let ((r (make-list (length vlist))))
                              (dotimes (i (length vlist))
                                (setf (elt r i) (scale (/ 1.0 255.0) (coerce (color-category20 i) float-vector))))
                              r)
                          (make-list (length vlist) :initial-element c)))
		    triface-vertices-list body-colors))
	   (points
	    (mapcar #'(lambda (v)
			(ros::pos->tf-point (send obj :inverse-transform-vector v)))
		    (apply #'append triface-vertices-list)))
	   (colors
	    (mapcar #'(lambda (c)
			(if (derivedp c gl::colormaterial) (setq c (send c :diffuse))) ;; jsk
			(vector->rgba c a))
		    (apply #'append color-list)))
	   )
      (send msg :frame_locked t)
      (send msg :pose (ros::coords->tf-pose (if coords coords (send obj :worldcoords))))
      (send msg :scale (ros::pos->tf-translation
			(float-vector 1000 1000 1000)))
      (send msg :points points)
      (send msg :color  (vector->rgba (float-vector 0 0 0) a))
      (send msg :colors colors)
      (if ns (send msg :ns ns))
      (if lifetime (send msg :lifetime (ros::time lifetime)))
      msg)))

(defun wireframe->marker-msg (wf header &rest args)
  "Convert euslisp object to visualization_msgs::Marker as wireframe"
  (apply #'line-list->marker-msg (mapcan #'(lambda (eds) (send eds :vertices)) (send wf :edges)) header args))

(defun text->marker-msg (str c header
			   &key
			   ((:color col) (float-vector 1 1 1))
			   ((:alpha a) 1.0)
			   ((:id idx) 0)
			   ((:scale sc) 100.0) ns lifetime)
  "Convert text to visualization_msgs::Marker"
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*TEXT_VIEW_FACING*
                       :header header
                       :id idx)))
    (send msg :pose (ros::coords->tf-pose c))
    (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
    (send msg :color (vector->rgba col a))
    (send msg :text str)
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun coords->marker-msg (coords header &key (size 100) (width 10) (id 0) ns lifetime)
  "Convert coordinates to visualization_msgs::Marker"
  (let* ((msg (instance visualization_msgs::Marker :init
                        :type visualization_msgs::Marker::*LINE_LIST*
                        :header header
                        :id id))
         (points (mapcar #'(lambda (p)
                             (ros::pos->tf-point (scale size p)))
                         (list (float-vector 0 0 0)
                               (float-vector 1 0 0)
                               (float-vector 0 0 0)
                               (float-vector 0 1 0)
                               (float-vector 0 0 0)
                               (float-vector 0 0 1))))
         (colors (mapcar #'(lambda (c) (vector->rgba c 1.0))
                         (list (float-vector 1 0 0) (float-vector 1 0 0)
                               (float-vector 0 1 0) (float-vector 0 1 0)
                               (float-vector 0 0 1) (float-vector 0 0 1)))))
    (send msg :pose (ros::coords->tf-pose coords))
    (send msg :scale (ros::pos->tf-translation
                      (float-vector width width width)))
    (send msg :points points)
    (send msg :color  (vector->rgba (float-vector 0 0 0) 1.0))
    (send msg :colors colors)
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun mesh->marker-msg
  (cds mesh_resource header
       &key ((:color col) (float-vector 1 1 1)) ((:scale sc) 1000)
       ((:id idx) 0) ((:mesh_use_embedded_materials use_embedded) t)
       (alpha 1.0) ns lifetime)
    "Convert mesh resources to visualization_msgs::Marker"
    (let* ((msg (instance visualization_msgs::Marker :init
			  :type visualization_msgs::Marker::*MESH_RESOURCE*
			  :header header :id idx)))
      (send msg :mesh_resource mesh_resource)
      (send msg :mesh_use_embedded_materials use_embedded)
      (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
      (send msg :color (vector->rgba col alpha))
      (send msg :frame_locked t)
      (send msg :pose (ros::coords->tf-pose cds))
      (if ns (send msg :ns ns))
      (if lifetime (send msg :lifetime (ros::time lifetime)))
      msg))

(defun pointcloud->marker-msg (pc header
                                  &key ((:color col) (float-vector 1.0 0 0))
                                  ((:alpha a) 1.0)
                                  (use-color) (point-width 0.01) (point-height 0.01)
                                  ((:id idx) 0) ns lifetime)
  "Convert pointcloud data to visualization_msgs::Marker"
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*POINTS*
                       :header header
                       :id idx)))

    (send msg :pose (ros::coords->tf-pose (send pc :worldcoords)))
    (send msg :color (vector->rgba col a))
    (send msg :scale :x point-width)
    (send msg :scale :y point-height)
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))

    (let ((mat (send pc :points))
          (p (float-vector 0 0 0)) lst)
      (dotimes (i (send pc :size))
        (c-matrix-row mat i p)
        (push (instance geometry_msgs::Point :init
                        :x (* 0.001 (elt p 0)) :y (* 0.001 (elt p 1))
                        :z (* 0.001 (elt p 2))) lst))
      (send msg :points lst))

    (when (and use-color (send pc :colors))
      (let ((mcol (send pc :points))
            (c (float-vector 0 0 0)) lst)
        (dotimes (i (send pc :size))
          (c-matrix-row mcol i c)
          (push (instance std_msgs::ColorRGBA :init
                          :r (elt c 0) :g (elt c 1) :b (elt c 2) :a a) lst))
        (send msg :colors lst)))
    msg))

(defun eusobj->marker-msg (eusgeom header &rest args &key (wireframe)
                                    &allow-other-keys)
  "Convert any euslisp object to visualization_msgs::Marker"
  (cond
   (wireframe
    (apply #'wireframe->marker-msg eusgeom header args))
   ((derivedp eusgeom body)
    (let ((tp (car (send eusgeom :body-type))))
      (case tp
        (:cube (apply #'cube->marker-msg eusgeom header args))
        (:cylinder (apply #'cylinder->marker-msg eusgeom header args))
        (:gdome (apply #'sphere->marker-msg eusgeom header args))
        (t (apply #'object->marker-msg eusgeom header args)))
      ))
   ((derivedp eusgeom bodyset)
    (apply #'object->marker-msg eusgeom header args))
   ((and (listp eusgeom) (derivedp (car eusgeom) line))
    (apply #'line-list->marker-msg (mapcan #'(lambda (l) (send l :vertices)) eusgeom) header args))
   ((and (listp eusgeom) (derivedp (car eusgeom) float-vector))
    (apply #'line->marker-msg eusgeom header args))
   ((derivedp eusgeom line)
    (apply #'line->marker-msg (send eusgeom :vertices) header args))
   ((and (listp eusgeom) (derivedp (car eusgeom) face))
    (apply #'faces->marker-msg eusgeom header args))
   ((derivedp eusgeom face)
    (apply #'faces->marker-msg (list eusgeom) header args))
   ((derivedp eusgeom pointcloud)
    (apply #'pointcloud->marker-msg eusgeom header args))
   ((derivedp eusgeom coordinates)
    (apply #'coords->marker-msg (send eusgeom :worldcoords) header args))
   ((stringp eusgeom)
    (apply #'string->marker-msg eusgeom header args))
   ))

;;http://www.ros.org/wiki/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29
;;arg type : coords -> Pose/Orientation way
;;         : 3d float-vector list -> Start/End Points way
(defun arrow->marker-msg (arg header
			      &key ((:color col) (float-vector 1.0 0 0))
			           ((:alpha a) 1.0)
			           ((:id idx) 0)
			           ((:scale sc) nil) ns lifetime)
  "Convert arrow object to visualization_msgs::Marker"
  (let ((msg (instance visualization_msgs::Marker :init
		       :type visualization_msgs::Marker::*ARROW*
		       :header header
		       :id idx)))
    (if (listp arg)
	(progn
	  (send msg :points (mapcar #'(lambda(p)(ros::pos->tf-point p)) arg))
	  (unless sc
	    (setq sc (float-vector 100.0 100.0 0))))
      (progn
	(send msg :pose (ros::coords->tf-pose arg))
	(unless sc
	  (setq sc (float-vector 1000.0 100.0 100.0)))))
    (send msg :scale (ros::pos->tf-translation sc))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))


;; visualization_msgs::Marker -> eus shape object

(defun marker-msg->shape (msg &rest args)
  "Convert visualization_msgs::Marker to euslisp object"
  (let ((type (send msg :type)))
    (cond
     ((eq type visualization_msgs::Marker::*CUBE*)     (marker-msg->shape/cube msg))
     ((eq type visualization_msgs::Marker::*CYLINDER*) (marker-msg->shape/cylinder msg))
     ((eq type visualization_msgs::Marker::*SPHERE*)   (marker-msg->shape/sphere msg))
     ((eq type visualization_msgs::Marker::*CUBE_LIST*)
      (apply #'marker-msg->shape/cube_list msg args))
     )))

(defun marker-msg->shape/cube (msg)
  "Convert visualization_msgs::Marker to euslisp cube object"
  (let* ((scale (ros::tf-point->pos (send msg :scale)))
         (rgba (send msg :color))
         (cb (make-cube (elt scale 0) (elt scale 1) (elt scale 2)
                        :coords (ros::tf-pose->coords (send msg :pose)))))
    (setf (get cb :face-color)
          (float-vector (send rgba :r)
                        (send rgba :g)
                        (send rgba :b)))
    (gl::transparent cb (send rgba :a))
    cb))

(defun marker-msg->shape/cylinder (msg)
  "Convert visualization_msgs::Marker to euslisp cylinder object"
  (let* ((scale (ros::tf-point->pos (send msg :scale)))
         (rgba (send msg :color))
         (radius (/ (elt scale 0) 2.0))
         (height (elt scale 2))
         (cyl (make-cylinder radius
                             height
                             :coords (ros::tf-pose->coords (send msg :pose)))))
    (send cyl :translate (float-vector 0 0 (- (/ height 2.0))))
    (setf (get cyl :face-color)
          (float-vector (send rgba :r)
                        (send rgba :g)
                        (send rgba :b)))
    (gl::transparent cyl (send rgba :a))
    cyl))

(defun marker-msg->shape/sphere (msg)
  "Convert visualization_msgs::Marker to euslisp sphere object"
  (let* ((scale (ros::tf-point->pos (send msg :scale)))
         (rgba (send msg :color))
         (sp (make-sphere (/ (elt scale 0) 2.0)
                          :coords (ros::tf-pose->coords (send msg :pose)))))
    (setf (get sp :face-color)
          (float-vector (send rgba :r)
                        (send rgba :g)
                        (send rgba :b)))
    (gl::transparent sp (send rgba :a))
    sp))

(defun marker-msg->shape/cube_list (msg &key ((:pointcloud pt)))
  "Convert visualization_msgs::Marker to eusliss list of cube objects"
  (let* ((scale (ros::tf-point->pos (send msg :scale)))
         (rgba (send msg :color))
         (cds (ros::tf-pose->coords (send msg :pose)))
         (points (send msg :points))
         (colors (send msg :colors)))
    (cond
     (pt
      (let ((p (instance pointcloud :init
                         :point-color (float-vector (send rgba :r)
                                                    (send rgba :g)
                                                    (send rgba :b))
                         :points (mapcar #'(lambda (p)
                                             (ros::tf-point->pos p))
                                         points)
                         :colors (mapcar #'(lambda (c)
                                             (float-vector (send c :r)
                                                           (send c :g)
                                                           (send c :b)))
                                         colors))))
        p))
     (colors
      (mapcar
       #'(lambda (pt cl)
           (let ((cb (make-cube (elt scale 0) (elt scale 1) (elt scale 2))))
             (send cb :translate (ros::tf-point->pos pt))
             (setf (get cb :face-color)
                   (float-vector (send cl :r)
                                 (send cl :g)
                                 (send cl :b)))
             cb)) points colors))
     (t
      (mapcar
       #'(lambda (pt)
           (let ((cb (make-cube (elt scale 0) (elt scale 1) (elt scale 2))))
             (send cb :translate (ros::tf-point->pos pt))
             (setf (get cb :face-color)
                   (float-vector (send rgba :r)
                                 (send rgba :g)
                                 (send rgba :b)))
             cb)) points)))
    ))

;;
;; Shape
;;
(ros::roseus-add-msgs "shape_msgs")

;; eus shape object -> shape_msgs::SolidPrimitive

(defun cube->shape-msg (cb)
  "Convert cube object to shape_msgs::SolidPrimitive"
  (instance shape_msgs::SolidPrimitive
            :init :type shape_msgs::SolidPrimitive::*BOX*
            :dimensions (float-vector
                         (/ (x-of-cube cb) 1000.0)
                         (/ (y-of-cube cb) 1000.0)
                         (/ (z-of-cube cb) 1000.0))))

(defun sphere->shape-msg (sp)
  "Convert sphere object to shape_msgs::SolidPrimitive"
  (instance shape_msgs::SolidPrimitive
            :init :type shape_msgs::SolidPrimitive::*SPHERE*
            :dimensions (float-vector
                         (/ (radius-of-sphere sp) 1000.0))))

(defun cylinder->shape-msg (cyl)
  "Convert cylinder object to shape_msgs::SolidPrimitive"
  (instance shape_msgs::SolidPrimitive
            :init :type shape_msgs::SolidPrimitive::*CYLINDER*
            :dimensions (float-vector
                         (/ (height-of-cylinder cyl) 1000.0)
                         (/ (radius-of-cylinder cyl) 1000.0))))

;; shape_msgs::SolidPrimitive -> eus shape object

(defun shape-msg->shape (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp object"
  (let ((type (send msg :type)))
    (cond
     ((eq type shape_msgs::SolidPrimitive::*BOX*)
      (shape-msg->cube msg))
     ((eq type shape_msgs::SolidPrimitive::*SPHERE*)
      (shape-msg->sphere msg))
     ((eq type shape_msgs::SolidPrimitive::*CYLINDER*)
      (shape-msg->cylinder msg))
     (t
      (error "unknown type ~A" type)))))

(defun shape-msg->cube (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp cube object"
  (let* ((scale (send msg :dimensions))
         (cb (make-cube (* (elt scale 0) 1000)
                        (* (elt scale 1) 1000)
                        (* (elt scale 2) 1000))))
    cb))

(defun shape-msg->sphere (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp sphere object"
  (let* ((scale (send msg :dimensions))
         (cp (make-sphere (* (elt scale 0) 1000))))
    cp))

(defun shape-msg->cylinder (msg)
  "Convert shape_msgs::SolidPrimitive to euslisp cylinder object"
  (let* ((scale (send msg :dimensions))
         (height (* (elt scale 0) 1000))
         (radius (* (elt scale 1) 1000))
         (cyl (make-cylinder radius height)))
    cyl))

;;
;; for pointcloud
;;
(defun make-ros-msg-from-eus-pointcloud (pcloud &key (with-color :rgb)
                                                (with-normal t) (frame "/sensor_frame"))
  "Convert from pointcloud in eus to sensor_msgs::PointCloud2 in ros"
  (let ((size (send pcloud :size))
        fld-lst hdr msg psize raw_data
        (width (send pcloud :width))
        (height (send pcloud :height))
        w h)
    (cond
     ((and width height)
      (setq w width h height)
      (unless (= size (* w h))
        (setq w size h 1)))
     (t
      (setq w size h 1)))

    (push (instance sensor_msgs::PointField :init :name "x"
                    :offset 0 :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
          fld-lst)
    (push (instance sensor_msgs::PointField :init :name "y"
                    :offset 4 :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
          fld-lst)
    (push (instance sensor_msgs::PointField :init :name "z"
                    :offset 8 :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
          fld-lst)

    (cond
     ((and (eq with-color :r-g-b) (send pcloud :colors))
      (push (instance sensor_msgs::PointField :init :name "r"
                      :offset 12 :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (push (instance sensor_msgs::PointField :init :name "g"
                      :offset 16 :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (push (instance sensor_msgs::PointField :init :name "b"
                      :offset 20 :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (setq psize 24))
     ((and with-color (send pcloud :colors))
      (push (instance sensor_msgs::PointField :init :name "rgb"
                      :offset 12 :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (setq psize 16))
     (t (setq psize 12)))

    (when (and with-normal (send pcloud :normals))
      (push (instance sensor_msgs::PointField :init :name "normal_x"
                      :offset (+ psize 0) :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (push (instance sensor_msgs::PointField :init :name "normal_y"
                      :offset (+ psize 4) :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (push (instance sensor_msgs::PointField :init :name "normal_z"
                      :offset (+ psize 8) :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (push (instance sensor_msgs::PointField :init :name "curvature"
                      :offset (+ psize 12) :datatype sensor_msgs::PointField::*FLOAT32* :count 1)
            fld-lst)
      (incf psize 16))

    (nreverse fld-lst)
    (setq raw_data (make-string (* size psize)))

    (convert-pointcloud-msg2 raw_data size psize
                             (send pcloud :points)
                             (if (and with-color (send pcloud :colors)) (send pcloud :colors))
                             (if (and with-normal (send pcloud :normals)) (send pcloud :normals)))

    (setq hdr (instance std_msgs::header :init
               :stamp (ros::time-now) :frame_id frame))

    (setq msg (instance sensor_msgs::PointCloud2 :init
                        :header hdr
                        :width w
                        :height h
                        :fields fld-lst
                        :point_step psize ;; point size by byte
                        :row_step   (* w psize) ;; row size by byte
                        :data raw_data
                        :is_dense 1
                        ))
    msg
    ))

(defun make-eus-pointcloud-from-ros-msg (msg &key (pcloud) (remove-nan))
  "Convert from sensor_msgs::PointCloud2 in ros to pointcloud in eus"
  (let* ((f-lst (send msg :fields))
         (w (send msg :width))
         (h (send msg :height))
         (step (send msg :point_step))
         (data (send msg :data))
         (size (* w h))
         mat cmat nmat
         use-point use-color use-normal
         px py pz nx ny nz prgb
         (field-names (vector "x" "y" "z" "rgb" "normal_x" "normal_y" "normal_z"))
         (field-indices (make-array (length field-names) :element-type integer-vector :initial-element -1))
         )

    (unless (and f-lst (/= (* w h) 0))
      (warn "empty sensor_msgs::PointCloud2 message~%")
      (return-from make-eus-pointcloud-from-ros-msg pcloud))

    (let (fld)
      (dotimes (i (length field-names))
        (when (setq fld (find-if #'(lambda (flx) (string= (send flx :name) (elt field-names i))) f-lst))
          (setf (elt field-indices i) (send fld :offset)))))

    (if (>= (elt field-indices 0) 0)
        (setq mat (make-matrix size 3)))
    (if (>= (elt field-indices 3) 0)
        (setq cmat (make-matrix size 3)))
    (if (>= (elt field-indices 4) 0)
        (setq nmat (make-matrix size 3)))

    (convert-msg2-pointcloud data step size
                             mat (elt field-indices 0) (elt field-indices 1) (elt field-indices 2)
                             nmat (elt field-indices 4) (elt field-indices 5) (elt field-indices 6)
                             cmat (elt field-indices 3)
                             (if (eq remove-nan :replace) 1 0))

    (cond
     (pcloud
      (if mat (send pcloud :points mat))
      (if cmat (send pcloud :colors cmat))
      (if nmat (send pcloud :normals nmat))
      (send pcloud :size-change w h))
     (t
      (setq pcloud (instance pointcloud :init :width w :height h
                          :points mat :colors cmat :normals nmat))))

    (when (and remove-nan (not (eq remove-nan :replace)))
      (let (index-lst
            (p (instantiate float-vector 3))
            (n (if nmat (instantiate float-vector 3))))
        (dotimes (i size)
          (c-matrix-row mat i p)
          (if n (c-matrix-row nmat i n))
          (if (and (not (c-isnan (elt p 0)))
                   (if n (not (c-isnan (elt n 0))) t))
              (push i index-lst)))
        (nreverse index-lst)
        (send pcloud :filter-with-indices index-lst)
        ))
    pcloud
    ))

(defun dump-pointcloud-to-pcd-file (fname pcloud &key (rgb :rgb) (binary) (scale 0.001))
  "Write pointcloud data to pcd file"
  (let ((pmat (send pcloud :points))
        (cmat (send pcloud :colors))
        (nmat (send pcloud :normals))
        (cnt 0)
        (f (open fname :direction :output)))
    (format f "# .PCD v.7 - Point Cloud Data file format~%")
    (format f "VERSION .7~%") ;;;;
    ;; FIELDS
    (format f "FIELDS x y z")
    (if cmat (case rgb
               (:r-g-b (format f " r g b"))
               (:rgb (format f " rgb"))
               (t (format f " rgba"))))
    (if nmat (format f " normal_x normal_y normal_z curvature"))
    ;; SIZE
    (format f "~%SIZE 4 4 4")
    (if cmat (case rgb
               (:r-g-b (format f " 4 4 4"))
               (:rgb (format f " 4"))
               (t (format f " 4"))))
    (if nmat (format f " 4 4 4 4"))
    ;; TYPE
    (format f "~%TYPE F F F")
    (incf cnt 3)
    (if cmat (case rgb
               (:r-g-b (format f " F F F") (incf cnt 3))
               (:rgb (format f " F") (incf cnt 1))
               (t (format f " U") (incf cnt 1))))
    (when nmat (format f " F F F F") (incf cnt 4))
    (format f "~%COUNT")
    (dotimes (i cnt) (format f " 1"))
    (format f "~%WIDTH ~A~%" (send pcloud :size))
    (format f "HEIGHT 1~%")
    (format f "VIEWPOINT 0 0 0 1 0 0 0~%")
    (format f "POINTS ~A~%" (send pcloud :size))
    (format f "DATA ascii~%") ;; binary mode is not implemented
    (dotimes (i (send pcloud :size))
      (let ((p (matrix-row pmat i))
            (c (if cmat (matrix-row cmat i)))
            (n (if nmat (matrix-row nmat i))))
        (if scale (scale scale p p))
        (format f "~8,8F ~8,8F ~8,8F" (elt p 0) (elt p 1) (elt p 2))
        (cond
         ((and c (eq rgb :r-g-b))
          (format f " ~8,8F ~8,8F ~8,8F" (elt c 0) (elt c 1) (elt c 2)) ) ;; not implimented
         ((and c (eq rgb :rgb))
          (let ((rgba (+ (* 256 256 (floor (* 255.0 (elt c 0))))
                         (* 256 (floor (* 255.0 (elt c 1))))
                         (floor (* 255.0 (elt c 2)))))
                (flt (make-string  4)))
            (sys::poke rgba flt 0 :integer)
            (format f " ~20,20e" (sys::peek flt 0 :float))
            ))
         (c (let ((rgba (+ (* 256 256 (floor (* 255.0 (elt c 0))))
                           (* 256 (floor (* 255.0 (elt c 1))))
                           (floor (* 255.0 (elt c 2))))))
              (format f " ~D" rgba))))
        (if  n (format f " ~8,8F ~8,8F ~8,8F 0.0" (elt n 0) (elt n 1) (elt n 2)))
        (format f "~%")))
    (close f)
    ))

(defun read-header-symbol (str sym)
  (when (eq sym (read-from-string str))
    (let ((s (make-string-input-stream str))
          elm ret)
      (read s) ;; eq sym
      (while (setq elm (read s nil nil))
        (push elm ret))
      (setq ret (nreverse ret))
      )))
(defun read-pcd-file (fname &key (scale 1000.0))
  "Write read pcd file and construct pointcloud object"
  (let (str
        version pfields
        psize ptype pcount
        width height
        vp points dtype)
    (with-open-file
     (f fname)
     ;; read header
     (while (setq str (read-line f))
       (unless (= (elt str 0) #\#)
         (let ((s (make-string-input-stream str))
               elem lst)
           (while (setq elem (read s nil nil))
             (push elem lst))
           (setq lst (nreverse lst))
           (case
            (car lst)
            ('VERSION
             (setq version (cadr lst)))
            ('FIELDS
             (setq pfields (cdr lst)))
            ('SIZE
             (setq psize (cdr lst)))
            ('TYPE
             (setq ptype (cdr lst)))
            ('COUNT
             (setq pcount (cdr lst)))
            ('WIDTH
             (setq width (cadr lst)))
            ('HEIGHT
             (setq height (cadr lst)))
            ('VIEWPOINT
             (setq vp (cdr lst)))
            ('POINTS
             (setq points (cadr lst)))
            ('DATA
             (setq dtype (cadr lst))
             (return))
            (t
             (warn ";; unknown symbol ~A~%" (car lst))
             )))))
#|
     (when debug
       (pprint version)
       (pprint pfields)
       (pprint psize)
       (pprint ptype)
       (pprint pcount)
       (pprint width)
       (pprint height)
       (pprint vp)
       (pprint points)
       (pprint dtype))
|#
     ;; read data
     ;; size check
     (let (off-lst
           pbytes bytes adtype
           xidx yidx zidx xoff yoff zoff xtp ytp ztp pmat
           frgb rgba rgboff rgbtp cmat rgbidx
           nx ny nz nxoff nyoff nzoff nxtp nytp nztp nmat)

       (setq pbytes (apply #'+ psize))
       (setq bytes (* pbytes points))

       (setq adtype
             (mapcar #'(lambda (sz tp)
                         (case tp
                               ('I (case sz (1 :char) (2 :short) (4 :integer)))
                               ('U (case sz (1 :char) (2 :short) (4 :integer)))
                               ('F (case sz (4 :float) (8 :double))))) psize ptype))

       (let ((cntr 0))
         (dotimes (i (length psize))
           (push cntr off-lst)
           (incf cntr (elt psize i))))
       (setq off-lst (nreverse off-lst))

       ;;(setq xidx (position 'x pfields))
       ;;(setq yidx (position 'y pfields))
       ;;(setq zidx (position 'z pfields))
       (setq pfields (mapcar #'(lambda (fsym) (symbol-string fsym)) pfields))
       (setq xidx (position "X" pfields :test #'string=))
       (setq yidx (position "Y" pfields :test #'string=))
       (setq zidx (position "Z" pfields :test #'string=))

       (when (and xidx yidx zidx)
         (setq xoff (elt off-lst xidx))
         (setq xtp (elt adtype xidx))
         (setq yoff (elt off-lst yidx))
         (setq ytp (elt adtype yidx))
         (setq zoff (elt off-lst zidx))
         (setq ztp (elt adtype zidx))
         (setq pmat (make-matrix points 3)))

       (setq frgb (position 'rgb pfields))
       (setq rgba (position 'rgba pfields))
       (when (or frgb rgba)
         (setq rgbidx (if frgb frgb rgba))
         (setq rgboff (elt off-lst rgbidx))
         (setq rgbtp (elt adtype rgbidx))
         (setq cmat (make-matrix points 3)))

       (setq nx (position 'normal_x pfields))
       (setq ny (position 'normal_y pfields))
       (setq nz (position 'normal_z pfields))
       (when (and nx ny nz)
         (setq nxoff (elt off-lst nx))
         (setq nxtp (elt adtype nx))
         (setq nyoff (elt off-lst ny))
         (setq nytp (elt adtype ny))
         (setq nzoff (elt off-lst nz))
         (setq nztp (elt adtype nz))
         (setq nmat (make-matrix points 3)))

       (cond
        ((eq dtype 'binary)
         ;; binary read
         (let ((buf (make-string pbytes))
               (vec (instantiate float-vector 3))
               addr)
           (setq addr (+ (sys::address buf) lisp::sizeof-header-offset))
           (dotimes (i points)
             (send f :read-bytes pbytes buf)
             (when pmat
               (setf (elt vec 0) (sys::peek (+ addr xoff) xtp))
               (setf (elt vec 1) (sys::peek (+ addr yoff) ytp))
               (setf (elt vec 2) (sys::peek (+ addr zoff) ztp))
               (c-matrix-row pmat i vec t))
             (when nmat
               (setf (elt vec 0) (sys::peek (+ addr nxoff) nxtp))
               (setf (elt vec 1) (sys::peek (+ addr nyoff) nytp))
               (setf (elt vec 2) (sys::peek (+ addr nzoff) nztp))
               (c-matrix-row nmat i vec t))
             (when cmat
               (cond
                ((eq rgbtp :float)
                 (let ((fval (sys::peek (+ addr rgboff) :integer)))
                   (setf (elt vec 2) (/ (ldb fval 0 8) 255.0))
                   (setf (elt vec 1) (/ (ldb fval 8 8) 255.0))
                   (setf (elt vec 0) (/ (ldb fval 16 8) 255.0))
                   ))
                (t
                 (let ((ival (sys::peek (+ addr rgboff) rgbtp)))
                   (setf (elt vec 2) (/ (ldb ival 0 8) 255.0))
                   (setf (elt vec 1) (/ (ldb ival 8 8) 255.0))
                   (setf (elt vec 0) (/ (ldb ival 16 8) 255.0))
                   )))
               (c-matrix-row cmat i vec t)))))
         (t ;; ascii
          (let (
                (vec (instantiate float-vector 3))
                (rdvec (instantiate vector (length pfields)))
                )
            (dotimes (i points)
              (dotimes (j (length pfields))
                (setf (elt rdvec j) (read f)))
              (when pmat
                (setf (elt vec 0) (elt rdvec xidx))
                (setf (elt vec 1) (elt rdvec yidx))
                (setf (elt vec 2) (elt rdvec zidx))
                (c-matrix-row pmat i vec t))
              (when nmat
                (setf (elt vec 0) (elt rdvec nx))
                (setf (elt vec 1) (elt rdvec ny))
                (setf (elt vec 2) (elt rdvec nz))
                (c-matrix-row nmat i vec t))
              (when cmat
                (cond
                 ((eq rgbtp :float)
                  (let ((fval (elt rdvec rgbidx))
                        (buf (make-string 4)) ival)
                    (sys::poke fval buf 0 :float)
                    (setq ival (sys::peek buf 0 :integer))
                    (setf (elt vec 2) (/ (ldb ival 0 8) 255.0))
                    (setf (elt vec 1) (/ (ldb ival 8 8) 255.0))
                    (setf (elt vec 0) (/ (ldb ival 16 8) 255.0))
                    ))
                 (t
                  (let ((ival (elt rdvec rgbidx)))
                    (setf (elt vec 2) (/ (ldb ival 0 8) 255.0))
                    (setf (elt vec 1) (/ (ldb ival 8 8) 255.0))
                    (setf (elt vec 0) (/ (ldb ival 16 8) 255.0))
                    )))
                (c-matrix-row cmat i vec t))))))
       (scale scale (array-entity pmat) (array-entity pmat))
       (instance pointcloud :init :points pmat
                 :colors cmat :normals nmat :height height :width width)
       ))
    ))
;;
;; misc function
;;
(ros::roseus-add-srvs "std_srvs")
(defun call-empty-service (srvname &key (wait nil) (timeout -1)) ;; If timeout is -1 (default), waits until the node is shutdown. If timeout is 0, returns immediately.
  "Call empty service and return t"
  (when (ros::wait-for-service srvname (if wait timeout 0))
    (ros::service-call srvname (instance std_srvs::EmptyRequest :init))
    t))

(defun one-shot-subscribe (topic-name mclass &key (timeout) (after-stamp) (unsubscribe t))
  "Subscribe message, just for once"
  (let (lmsg)
    (unless (ros::get-num-publishers topic-name)
      (cond
       (after-stamp
        (ros::subscribe topic-name mclass
                        #'(lambda (msg)
                            (let ((st (send msg :header :stamp)))
                              (when (> (send st :to-sec)
                                       (send after-stamp :to-sec))
                                (setq lmsg msg))))))
       (t
        (ros::subscribe topic-name mclass
                        #'(lambda (msg) (setq lmsg msg))))))
    (let ((finishtm (if timeout (ros::time-now))))
      (when finishtm
        (setq finishtm (ros::time+ finishtm (ros::time (/ timeout 1000.0)))))
      (while (not (and finishtm
                       (< (send (ros::time- finishtm (ros::time-now)) :to-Sec) 0)))
        (unix::usleep (* 50 1000))
        (ros::spin-once)
        (if lmsg (return))))
    (if unsubscribe (ros::unsubscribe topic-name))
    lmsg))

(defun one-shot-publish (topic-name msg &key (wait 500) (after-stamp) (unadvertise t))
  "Publish message just for once"
  (ros::advertise topic-name (class msg))
  (unix::usleep (round (* wait 1000)))
  (while after-stamp
    (let ((tm (ros::time-now)))
      (if (< (send (ros::time- after-stamp tm) :to-sec) 0)
          (return)))
    (unix::usleep 50000))
  (ros::publish topic-name msg)
  (unix::usleep (round (* wait 1000)))
  (if unadvertise (ros::unadvertise topic-name))
  msg)


(defun ros::set-dynamic-reconfigure-param (srv name type value)
  "set dynamic_reconfugre parameter."
  (let ((req (instance dynamic_reconfigure::ReconfigureRequest :init)))
    (cond ((eq type :double)
           (send req :config :doubles
                 (list (instance dynamic_reconfigure::DoubleParameter :init
                                 :name name :value value))))
          ((eq type :string)
           (send req :config :strs
                 (list (instance dynamic_reconfigure::StrParameter :init
                                 :name name :value value))))
          ((eq type :bool)
           (send req :config :bools
                 (list (instance dynamic_reconfigure::BoolParameter :init
                                 :name name :value value))))
          ((eq type :int)
           (send req :config :ints
                 (list (instance dynamic_reconfigure::IntParameter :init
                                 :name name :value value))))
          (t
           (error "unknown type ~A" type)))
    (ros::service-call (format nil "~A/set_parameters" srv) req)))

;; Class to synchronize multiple topics
;; like message_filters
(defclass exact-time-message-filter-callback-helper
  :super propertied-object
  :slots (deligate topic))

(defmethod exact-time-message-filter-callback-helper
  (:init (topic-info &key deligate-object)
    "topic-info := (topic message-type)
     deligate-object := exact-time-message-filter"
    (setq deligate deligate-object)
    (setq topic (car topic-info))
    (ros::subscribe (car topic-info) (cadr topic-info) #'send self :callback)
    )
  (:callback (msg)
    "just call :add-message method of deligate object"
    (send deligate :add-message topic msg))
  )

(defclass exact-time-message-filter
  :super propertied-object
  :slots (callback-helpers buffers buffer-length)
  :documentation "Exact time message filter synchronizes incoming channels by the timestamps contained in their headers, and outputs them in the form of a single callback that takes the same number of channels. This filter requres messages to have exactly the same timestamp in order to match. Your callback is only called if a message has been received on all specified channels with the same exact timestamp.

topic := ((topic message-type) (topic message-type) ...)

	;; sample
	(defclass image-caminfo-synchronizer
	  :super exact-time-message-filter)
	
	(defmethod image-caminfo-synchronizer
	  (:callback (image caminfo)
		(print (list image caminfo))
		(print (send-all (list image caminfo) :header :stamp))
		))
	(ros::roseus \"hoge\")
	(ros::roseus-add-msgs \"sensor_msgs\")
	;; test
	(setq hoge (instance image-caminfo-synchronizer :init
			 (list (list \"/multisense/left/image_rect_color\" sensor_msgs::Image)
			   (list \"/multisense/left/camera_info\" sensor_msgs::CameraInfo))))
	(ros::spin)
"
  )


(defmethod exact-time-message-filter
  (:init (topics &key ((:buffer-length abuffer-length) 50))
    "Create exact-time-message-filter instance"
    ;; initialize buffers for each topics
    (setq buffer-length abuffer-length)
    (setq buffers (mapcar #'(lambda (topic-info)
                              (cons (car topic-info) (make-list 0)))
                          topics))
    (setq callback-helpers
          (mapcar #'(lambda (topic-info)
                      (instance exact-time-message-filter-callback-helper
                                :init topic-info
                                :deligate-object self))
                  topics)))
  (:add-message (topic-name message)
    "add message to buffer and call callback if possible"
    (let ((buffer (cdr (assoc topic-name buffers))))
      (setf (cdr (assoc topic-name buffers)) (nconc buffer (list message)))
      ;; check the length of buffer
      (when (> (length buffer) buffer-length)
        (setf (cdr (assoc topic-name buffers))
              (subseq (cdr (assoc topic-name buffers))
                      (- (length (cdr (assoc topic-name buffers))) buffer-length)
                      (length (cdr (assoc topic-name buffers))))))
      ;; (format t "buffer length ~A~%" (length buffer))
      (send self :call-callback-if-possible (send message :header :stamp))
      ))
  (:call-callback-if-possible (stamp)
    "call synchronized callback if messages can be found
with the same timestamp"
    (let ((candidates
           (mapcar #'(lambda (buffer)
                       (find-if #'(lambda (msg)
                                    (let ((diff (ros::time- (send msg :header :stamp) stamp)))
                                      (= (send diff :to-sec) 0)))
                                buffer))
                   (mapcar #'cdr buffers))))
      ;; check if all topic is available
      (when (= (length (remove-if #'null candidates)) 
               (length candidates))
        (send* self :callback candidates))
      ))
  )

#|
;; sample
(defclass image-caminfo-synchronizer
  :super exact-time-message-filter)

(defmethod image-caminfo-synchronizer
  (:callback (image caminfo)
    (print (list image caminfo))
    (print (send-all (list image caminfo) :header :stamp))
    ))
(ros::roseus "hoge")
(ros::roseus-add-msgs "sensor_msgs")
;; test
(setq hoge (instance image-caminfo-synchronizer :init
                     (list (list "/multisense/left/image_rect_color" sensor_msgs::Image)
                           (list "/multisense/left/camera_info" sensor_msgs::CameraInfo))))
(ros::spin)
|#
