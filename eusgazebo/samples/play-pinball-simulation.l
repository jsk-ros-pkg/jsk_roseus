(load "package://eusgazebo/euslisp/eusgazebo.l")


(defun init-pinball-simulation
  ()

  ;; make and view eus model
  (load "package://eusgazebo/samples/ground-plane-object.l")
  (load "package://eusgazebo/samples/simple-ball-object.l")
  (load "package://eusgazebo/samples/domino-cube-object.l")
  (setq *floar* (ground-plane))
  (setq *ball* (simple-ball))
  (setq *bar1* (domino-cube))
  (setq *bar2* (domino-cube))
  (setq *wall1* (domino-cube))
  (setq *wall2* (domino-cube))
  (objects (list *ball* *floar* *bar1* *bar2* *wall1* *wall2*))

  ;; overwrite mass property
  (overwrite-mass-property *ball* (float-vector 0 0 100) 1000 (list 1e8 1e8 1e8))

  ;; generate eusgazebo instance
  (setq *eusgazebo-server* (instance eusgazebo :init))
  (send *eusgazebo-server* :add-model *floar* :static t)
  (send *eusgazebo-server* :add-model *wall1* :static t)
  (send *eusgazebo-server* :add-model *wall2* :static t)
  (send *eusgazebo-server* :add-model *bar1* :static t)
  (send *eusgazebo-server* :add-model *bar2* :static t)
  (send *eusgazebo-server* :add-model *ball*)
  (unix::usleep (* 1000 1000)))


(defun play-pinball-simulation
  ()

  ;; start simulation
  ;; pause simulation
  (send *eusgazebo-server* :pause-sim)
  ;; set eus model pose
  (send *floar* :newcoords (make-coords :pos #f(0 0 2000) :rpy (list 0 (/ pi 10) 0)))
  (send *ball* :newcoords (make-coords :pos #f(-4000 0 5000) :rpy (list 0 0 0)))
  (send *wall1* :newcoords
	(send (make-coords :pos #f(-3000 1500 3500) :rpy (list 0 (/ pi 10) pi/2)) :rotate (deg2rad 40) :y :local))
  (send *wall2* :newcoords
	(send (make-coords :pos #f(0 -500 2500) :rpy (list 0 (/ pi 10) pi/2)) :rotate (deg2rad -40) :y :local))
  (send *bar1* :newcoords
	(send (make-coords :pos #f(2000 -1900 1800) :rpy (list 0 (/ pi 10) pi/2)) :rotate 0 :y :local))
  (send *bar2* :newcoords
	(send (make-coords :pos #f(2000 1900 1800) :rpy (list 0 (/ pi 10) pi/2)) :rotate pi :y :local))

  ;; apply eus pose to gazebo
  (send *eusgazebo-server* :eus2gzb)
  ;; unpause simulation
  (send *eusgazebo-server* :unpause-sim)
  (unix::usleep (* 500 1000))

  (do-until-key
   ;; gzb2eus
   (send *eusgazebo-server* :gzb2eus)
   (send *irtviewer* :draw-objects)

   ;; eus2gzb
   (send *bar1* :rotate (deg2rad 4) :y :local)
   (send *bar2* :rotate (deg2rad -4) :y :local)
   (send *eusgazebo-server* :eus2gzb *bar1*)
   (send *eusgazebo-server* :eus2gzb *bar2*)
   (when (< (elt (send *ball* :worldpos) 2) 1000)
     (send *ball* :newcoords (make-coords :pos #f(-4000 0 5000) :rpy (list 0 0 0)))
     (send *eusgazebo-server* :eus2gzb *ball*))

   ;; view
   (x::window-main-one)
   (unix::usleep (* 10 1000)))
  )
