;; overwrite mass property ;;;;;;;;;;;;;;;
(defun overwirte-mass-property
  (obj link-mass link-inertia-diag-elem)

  (dolist (link (send obj :links))  
    (overwrite-link-mass link link-mass)
    (overwrite-link-inertia link link-inertia-diag-elem))
  )

(defun overwirte-mass-property-if-too-small
  (obj)

  (let* ((link-weight-min 1e6) ;; [g]
	 (link-inertia-diag-min (list 1e6 1e6 1e6)) ;; [gmm^2]
	 )
    (dolist (link (send obj :links))
      (let* ((link-weight (slot link bodyset-link 'weight))
	     (link-inertia (slot (car (send obj :links)) bodyset-link 'inertia-tensor))
	     (link-inertia-diag (mapcar #'(lambda (i) (elt (matrix-row link-inertia i) i)) (list 0 1 2))))
	;; overwirte mass if too small
	(when (< link-weight link-weight-min)
	  (warning-message 1 "overwrite mass because original value is too small: ~a [g] -> ~a [g]~%" link-weight link-weight-min)
	  (overwrite-link-mass link link-weight-min))
	;; overwrite inertia if too small
	(let* ((link-inertia-diag-overwirte
		(mapcar #'max link-inertia-diag link-inertia-diag-min)))
	  (unless (equal link-inertia-diag link-inertia-diag-overwirte)
	    (warning-message 1 "overwrite inertia because original value is too small: ~a [gmm^2] -> ~a [gmm^2]~%"
			     link-inertia-diag link-inertia-diag-overwirte)
	    (overwrite-link-inertia link link-inertia-diag-overwirte)))
	))))

(defun overwrite-link-mass
  (link link-mass)

  (setf (slot link bodyset-link 'weight) link-mass))

(defun overwrite-link-inertia
  (link link-inertia-diag-elem) ;; link-inertia-diag-elem shoud be 3-length list
  ;; this function handles only diag element

  (dolist (idx (list 0 1 2))
    (let* ((link-inertia (slot link bodyset-link 'inertia-tensor))
	   (tmp-row (matrix-row link-inertia idx)))
      (setf (elt tmp-row idx) (elt link-inertia-diag-elem idx))
      (setf (matrix-row link-inertia idx) tmp-row))))


;; get mass property ;;;;;;;;;;;;;;;
(defun get-link-mass
  (link)

  (slot link bodyset-link 'weight))

(defun get-link-inertia
  (link)

  (slot link bodyset-link 'inertia-tensor))

(defun print-obj-mass-property
  (obj)

  (warning-message 2 "whole mass: ~a [g]~%first link inertia: ~a [gmm^2]~%"
		   (send obj :weight) (get-link-inertia (car (send obj :links)))))


;; caluculate valid mass property automatically ;;;;;;;;;;;;;;;
(defun get-obj-bounding-box-min-max-point
  (obj)

  (let* ((min-point (list *inf* *inf* *inf*))
	 (min-point-list 
	  (mapcar #'(lambda (face) (send (send face :box) :minpoint))
		  (send obj :faces)))
	 (max-point (list *-inf* *-inf* *-inf*))
	 (max-point-list 
	  (mapcar #'(lambda (face) (send (send face :box) :maxpoint))
		  (send obj :faces))))
    (dolist (p min-point-list)
      (dolist (idx (list 0 1 2))
	(when (< (elt p idx) (elt min-point idx))
	  (setf (elt min-point idx) (elt p idx)))
	))

    (dolist (p max-point-list)
      (dolist (idx (list 0 1 2))
	(when (> (elt p idx) (elt max-point idx))
	  (setf (elt max-point idx) (elt p idx)))
	))
    (list min-point max-point)))
