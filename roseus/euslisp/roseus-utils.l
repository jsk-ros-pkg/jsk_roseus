;; this is a collection of utility functions and macros
;; for ros<->euslisp interface

(ros::roseus-add-msgs "sensor_msgs")

(defun make-camera-from-ros-camera-info-aux (pwidth pheight p frame-coords)
  (let* ((b (body+ (make-cube 40 30 30)
                   (send (make-cylinder 2 30) :rotate pi/2 :x)
                   (make-cylinder 13 25)))
         (fx (elt p 0))(fy (elt p 5))
         (cx (elt p 2))(cy (elt p 6))
         (tx (elt p 3))(ty (elt p 7))
         c (vd 1.0))
    (setq c (instance camera :init b :width pwidth :height pheight
                      :viewdistance vd))
    (send (c . viewing) :screen (/ (* vd pwidth 0.5) fx) (/ (* vd pheight 0.5) fy))
    (send c :newprojection
          (make-matrix 4 4
                       (list (float-vector fx  0 cx (* fx tx))
                             (float-vector  0 fy cy (* fy ty))
                             (float-vector  0  0  1  0)
                             (float-vector  0  0  0  1))))
    (send c :newcoords (send frame-coords :copy-worldcoords))
    (send c :translate (float-vector (- tx) (- ty) 0))
    (send (c . viewing) :translate (float-vector tx ty 0))
    (send frame-coords :assoc c)
    c))

(defun make-camera-from-ros-camera-info (msg)
  (let ((pwidth (send msg :width))
        (pheight (send msg :height))
        (p (send msg :P)))
    (make-camera-from-ros-camera-info-aux pwidth pheight p)))

