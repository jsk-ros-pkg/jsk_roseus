;; this is a collection of utility functions and macros
;; for ros<->euslisp interface

;; load c-binding library
(load-library "roseus_c_util.so" '("roseus_c_util"))

;; Sensors
(ros::roseus-add-msgs "sensor_msgs")

(defun make-camera-from-ros-camera-info-aux (pwidth pheight p frame-coords &rest args)
  (let* ((fx (elt p 0))(fy (elt p 5))
         (cx (elt p 2))(cy (elt p 6))
         (tx (elt p 3))(ty (elt p 7)))
    (apply #'make-camera-from-param :pwidth pwidth :pheight pheight
	   :fx fx :fy fy :cx cx :cy cy
	   :tx tx :ty ty :parent-coords frame-coords args)))

(defun make-camera-from-ros-camera-info (msg)
  (let ((pwidth (send msg :width))
        (pheight (send msg :height))
        (p (send msg :P)))
    ;; FIXME:: add coordinates of camera
    (make-camera-from-ros-camera-info-aux pwidth pheight p nil)))

;;
;; 3dpoint cloud
;;
(defun make-msg-from-3dpointcloud (points-list &key color-list (frame "/sensor_frame"))
  (let (points hdr msg)
    ;; make-points
    (dolist (length points-list)
      (push (instance geometry_msgs::Point32 :init) points))
    (mapcar #'(lambda (p pt)
		(send p :x (/ (elt pt 0) 1000.0))
		(send p :y (/ (elt pt 1) 1000.0))
		(send p :a (/ (elt pt 2) 1000.0)))
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

(defun faces->marker-msg (faces header &key
                                ((:color col) (float-vector 1 0 0))
                                ((:id idx) 0) ns lifetime)
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
    (send msg :color (vector->rgba (float-vector 1 0 0) 1.0))

    (send msg :points points)
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun object->marker-msg (obj header &key
			       coords
                               ((:color col) (float-vector 1 1 1))
			       ((:alpha a) 1.0)
                               ((:id idx) 0) ns lifetime)
  (let ((bodies (if (find-method obj :bodies) (send obj :bodies) (list obj))))
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
		    (mapcar #'body-to-faces bodies)))
	   (color-list
	    (mapcar #'(lambda (vlist c)
			(if (derivedp c gl::colormaterial) (setq c (send c :diffuse))) ;; jsk
			(make-list (length vlist) :initial-element c))
		    triface-vertices-list body-colors))
	   (points
	    (mapcar #'(lambda (v)
			(ros::pos->tf-point (send obj :inverse-transform-vector v)))
		    (apply #'append triface-vertices-list)))
	   (colors
	    (mapcar #'(lambda (c)
			(if (derivedp c gl::colormaterial) (setq c (send c :diffuse))) ;; jsk
			(vector->rgba c 1.0))
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
      msg))))

(defun text->marker-msg (str c header
			   &key
			   ((:color col) (float-vector 1 1 1))
			   ((:alpha a) 1.0)
			   ((:id idx) 0)
			   ((:scale sc) 100.0) ns lifetime)
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

(defun coords->marker-msg (coords header &key (size 1) (width 1) (id 0) ns lifetime)
  (let* ((msg (instance visualization_msgs::Marker :init
                        :type visualization_msgs::Marker::*LINE_LIST*
                        :header header
                        :id id))
         (points (mapcar #'(lambda (p)
                             (ros::pos->tf-point (scale (* 100 size) p)))
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
                      (float-vector width 0 0)))
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

;; visualization_msgs::Marker -> eus shape object

(defun marker-msg->shape (msg)
  (let ((type (send msg :type)))
    (print type)
    (print (numberp type))
    (case type
      (visualization_msgs::Marker::*CUBE*     (marker-msg->shape/cube msg))
      (visualization_msgs::Marker::*CYLINDER* (marker-msg->shape/cylinder msg))
      (visualization_msgs::Marker::*SPHERE*   (marker-msg->shape/sphere msg))
      )))

(defun marker-msg->shape/cube (msg)
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

;;
;; for pointcloud
;;
(defun make-ros-msg-from-eus-pointcloud (pcloud &key (with-color :rgb)
                                                (with-normal t) (frame "/sensor_frame"))
  "convert from pointcloud in eus to sensor_msgs::PointCloud2 in ros"
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
  "convert from sensor_msgs::PointCloud2 in ros to pointcloud in eus"
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
      (if nmat (send pcloud :normals nmat)))
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
  (let ((pmat (send pcloud :points))
        (cmat (send pcloud :colors))
        (nmat (send pcloud :normals))
        (f (open fname :direction :output)))
    (format f "# .PCD v.6 - Point Cloud Data file format~%")
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
    (if cmat (case rgb
               (:r-g-b (format f " F F F"))
               (:rgb (format f " F"))
               (t (format f " U"))))
    (if nmat (format f " F F F F"))

    (format f "~%WIDTH ~A~%" (send pcloud :size))
    (format f "HEIGHT 1~%")
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
                (setf (elt vec 0) (elt rdvec nxidx))
                (setf (elt vec 1) (elt rdvec nyidx))
                (setf (elt vec 2) (elt rdvec nzidx))
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
;; misc unction
;;
(ros::roseus-add-srvs "std_srvs")
(defun call-empty-service (srvname)
  (ros::wait-for-service srvname)
  (ros::service-call srvname (instance std_srvs::EmptyRequest :init)))
