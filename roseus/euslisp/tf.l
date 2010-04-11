(load-library "eustf.so" '("eustf"))
(pushnew :tf *features*)

(defun tf::make-transformation (from-id to-id)
  (let ((tf-ret (tf::_lookup-transform from-id to-id)))
    (make-coords :pos (float-vector
                       (* 1000.0 (elt tf-ret 0))
                       (* 1000.0 (elt tf-ret 1))
                       (* 1000.0 (elt tf-ret 2)))
                 :rot
                 (quaternion2matrix
                  (float-vector (elt tf-ret 3)
                                (elt tf-ret 4)
                                (elt tf-ret 5)
                                (elt tf-ret 6))))
    ))