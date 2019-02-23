(unless (find-package "BULLET") (make-package "BULLET" :nicknames '("BT")))
(in-package "BT")

(defvar *eusbullet-lib*
  (labels
      ((library_search
        (str &key colon-pos lib-path)
        (cond
         ((eq (length str) 0)
          (format t "~% libeusbullet.so not found~%")
          (exit -1))
         ((and (setq colon-pos (or (position #\: str) (length str)))
               (setq lib-path (subseq str 0 colon-pos))
               (setq lib-path
                     (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
                         (subseq lib-path 0 (- (length lib-path) 1))
                       lib-path))
               (probe-file (setq lib-path
                                 (format nil "~A/libeusbullet.so" lib-path))))
          (load-foreign lib-path))
         (t
          (library_search (subseq str (min (length str) (+ colon-pos 1))))))))
    (library_search (format nil "~A" (unix:getenv "LD_LIBRARY_PATH")))))


;; long callMakeSphereModel(double radius)
(defforeign make-sphere-model
  *eusbullet-lib*
  "callMakeSphereModel"
  (:float)
  :integer
  )

;; long callMakeBoxModel(double xsize, double ysize, double zsize)
(defforeign make-box-model
  *eusbullet-lib*
  "callMakeBoxModel"
  (:float :float :float)
  :integer
  )

;; long callMakeCylinderModel(double radius, double height)
(defforeign make-cylinder-model
  *eusbullet-lib*
  "callMakeCylinderModel"
  (:float :float)
  :integer
  )

;; long callMakeCapsuleModel(double radius, double height)
(defforeign make-capsule-model
  *eusbullet-lib*
  "callMakeCapsuleModel"
  (:float :float)
  :integer
  )

;; long callMakeMeshModel(double *verticesPoints, long numVertices)
(defforeign make-mesh-model
  *eusbullet-lib*
  "callMakeMeshModel"
  (:string :integer)
  :integer
  )

;; long callCalcCollisionDistance(long modelAddrA, long modelAddrB,
;;                                double *posA, double *quatA, double *posB, double *quatB,
;;                                double *dist, double *dir, double *pA, double *pB)
(defforeign calc-collision-distance
  *eusbullet-lib*
  "callCalcCollisionDistance"
  (:integer :integer
   :string :string :string :string
   :string :string :string :string)
  :integer
  )

;; long callSetMargin(long modelAddr, double margin)
(defforeign set-margin
  *eusbullet-lib*
  "callSetMargin"
  (:integer :float)
  :integer
  )

(defun make-model-from-body
    (b &key (csg (send b :csg)) (margin nil) m)
  (cond ((assoc :sphere csg)
         (setq m
               (make-sphere-model
                (* 1e-3 (user::radius-of-sphere b)))
               ))
        ((assoc :cube csg)
         (setq m
               (make-box-model
                (* 1e-3 (user::x-of-cube b))
                (* 1e-3 (user::y-of-cube b))
                (* 1e-3 (user::z-of-cube b)))
               ))
        ((assoc :cylinder csg)
         (setq m
               (make-cylinder-model
                (* 1e-3 (user::radius-of-cylinder b))
                (* 1e-3 (user::height-of-cylinder b)))
               ))
        (t
         (setq m
               (make-mesh-model
                (scale 1e-3 ;; [m]
                       (apply #'concatenate float-vector
                              (mapcar #'(lambda (v) (send b :inverse-transform-vector v)) (send b :vertices))))
                (length (send b :vertices))
                ))
         ))
  (when margin
    (bt::set-margin m margin))
  m)

(in-package "USER")

(defmethod cascaded-coords
  (:make-btmodel
   (&key (fat 0) vs m)
   (cond ((derivedp self body)
          (setq m
                (bt::make-model-from-body self :margin fat))
          )
         (t
          (setq vs (flatten (send-all (send self :bodies) :vertices)))
          (setq m
                (bt::make-mesh-model
                 (scale 1e-3 ;; [m]
                        (apply #'concatenate float-vector
                               (mapcar #'(lambda (v) (send self :inverse-transform-vector v)) vs)))
                 (length vs)
                 ))
          (bt::set-margin m fat)
          ))
   (setf (get self :btmodel) m)
   m)
  )

(defun bt-collision-distance
    (model1 model2 &key (fat 0) (fat2 nil) (qsize))
  "Calculate collision distance between model1 and model2 using Bullet.
   Return value is (list [distance] [nearest point on model1] [nearest point on model2]).
   If collision occurs, [distance] is 0 and nearest points are insignificant values.
   qsize argument is not used, just for compatibility with pqp-collision-distance."
  (let ((m1 (get model1 :btmodel))
        (m2 (get model2 :btmodel))
        (r1 (matrix2quaternion (send model1 :worldrot)))
        (t1 (scale 1e-3 (send model1 :worldpos))) ;; [m]
        (r2 (matrix2quaternion (send model2 :worldrot)))
        (t2 (scale 1e-3 (send model2 :worldpos))) ;; [m]
        (dist (float-vector 0))
        (dir (float-vector 0 0 0))
        (p1 (float-vector 0 0 0))
        (p2 (float-vector 0 0 0))
        r)
    (if (null fat2) (setq fat2 fat))
    (if (null m1) (setq m1 (send model1 :make-btmodel :fat fat)))
    (if (null m2) (setq m2 (send model2 :make-btmodel :fat fat2)))
    (bt::calc-collision-distance
     m1 m2 t1 r1 t2 r2
     dist dir p1 p2)
    (list (* 1e3 (elt dist 0)) (scale 1e3 p1) (scale 1e3 p2))
    ))

(defun bt-collision-check
    (model1 model2 &key (fat 0) (fat2 nil))
  "Check collision between model1 and model2 using Bullet.
   If return value is 0, no collision.
   Otherwise (return value is 1), collision."
  (if (> (elt (bt-collision-distance model1 model2 :fat fat :fat2 fat2) 0) 0) 0 1)
  )
