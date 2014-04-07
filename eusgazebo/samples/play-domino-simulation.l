(load "package://eusgazebo/euslisp/eusgazebo.l")


(defun init-domino-simulation
  ()

  ;; make and view eus model
  (load "package://eusgazebo/samples/domino-cube-object.l")
  (setq *cube1* (domino-cube))
  (setq *cube2* (domino-cube))
  (setq *cube3* (domino-cube))
  (setq *cube4* (domino-cube))
  (setq *cube5* (domino-cube))
  (objects (list *cube1* *cube2* *cube3* *cube4* *cube5*))

  ;; overwrite mass property
  (dolist (cube (list *cube1* *cube2* *cube3* *cube4* *cube5*))
    (overwrite-mass-property cube (float-vector 0 0 1000) 10000 (list 1e10 1e10 1e9)))

  ;; generate eusgazebo instance
  (setq *eusgazebo-server* (instance eusgazebo :init))
  (send *eusgazebo-server* :add-model *cube1*)
  (send *eusgazebo-server* :add-model *cube2*)
  (send *eusgazebo-server* :add-model *cube3*)
  (send *eusgazebo-server* :add-model *cube4*)
  (send *eusgazebo-server* :add-model *cube5* :static t)
  (unix::usleep (* 1000 1000)))


(defun play-domino-simulation
  ()

  ;; start simulation
  (progn
    ;; pause simulation
    (send *eusgazebo-server* :pause-sim)
    ;; set eus model pose
    (send *cube1* :newcoords (make-coords :pos #f(-3000 0 2000) :rpy (list 0 (/ pi 20) 0)))
    (send *cube2* :newcoords (make-coords :pos #f(-1500 0 2000) :rpy (list 0 0 0)))
    (send *cube3* :newcoords (make-coords :pos #f(0 0 2000) :rpy (list 0 0 0)))
    (send *cube4* :newcoords (make-coords :pos #f(1500 0 2000) :rpy (list 0 0 0)))
    (send *cube5* :newcoords (make-coords :pos #f(3000 0 0) :rpy (list 0 0 0)))
    ;; apply eus pose to gazebo
    (send *eusgazebo-server* :eus2gzb)
    ;; unpause simulation
    (send *eusgazebo-server* :unpause-sim)
    ;; view
    (dotimes (i 500)
      (send *eusgazebo-server* :gzb2eus)
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (unix::usleep (* 10 1000)))
    ;;(send *eusgazebo-server* :gzb2eus-loop)
    ))
