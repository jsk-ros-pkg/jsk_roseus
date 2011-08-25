;;
;; Convert from PDDL result (include failure recovery) to State Machine
;;
;; Author: Manabu Saito <saito@jsk.t.u-tokyo.ac.jp>
;;

;; TODO
;; start, goal state
;; for many arc
;; return value

(defun rmnl (str) ;; remove new line
  (map string #'(lambda(c)(if (= c #\\) #\_ c)) str))
;  str)

(defun convert-smach (gr)
  (let ((sm (instance state-machine :init)) act name sym goals
	(ns (send gr :name)))
    ;; add nodes
    (dolist (n (send gr :nodes))
      (setq act (send n :neighbor-action-alist)
	    name (rmnl (send n :name)))
      (cond ((null act) ;; no more action
	     (push name goals))
	    (t ;; last arc is correct action
	     (setq sym (read-from-string (caar (last act))))
	     (when (find-package (string-upcase ns))
	       (setf (elt sym 0)
		     (intern (string-upcase (car sym)) (find-package ns))))
	     (setq sym
		   `(lambda-closure nil 0 0 (x) (apply ',(car sym) ',(cdr sym))))
	     (send sm :add-node (instance state :init name sym)))
	    ))
    (send sm :goal-state goals)
    ;; add arcs
    (dolist (n (send gr :nodes))
      (setq act (send n :neighbor-action-alist)
	    name (rmnl (send n :name)))
      (cond ((null act))
	    (t
	     (send sm :add-transition name (rmnl (send (cdar (last act)) :name)) t)
	     (when (< 1 (length act))
	       (send sm :add-transition name (rmnl (send (cdar act) :name)) nil)))))

    ;; select a node as start-node
    ;; now this is not always true
    ;; (send sm :start-state (send (send gr :start-state) :name))
    (send sm :start-state (rmnl (send (car (last (send gr :nodes))) :name)))
    (send sm :reset-state)
    sm ))

;;
;; convert pddl-domain -> euslisp template of action definition
;;
(defun output-domain-template (filename domain)
  (with-open-file
   (fd filename :direction :output)
   (format fd ";;~%;; Please Fill in the Actions~%;; DOMAIN : ~a~%;;"
	   (send domain :name))
   (format fd "~%~%")
   ;;
   (pprint
    `(let ((ns ,(string-upcase (send *domain* :name))))
       (when (and ns (not (find-package ns)))
	 (make-package ns)))
    fd)
   (format fd "~%~%")
   ;;
  (dolist (act (send domain :action))
    (format fd "(defun ~a::~a~%  ~a~%t )~%"
	    (send domain :name) (send act :name)
	    (mapcan
	     #'(lambda(x)(if (= 1 (length x)) x (subseq x 0 (1- (length x)))))
	     (send act :parameters))
	    ))
  ;;
  (format fd "~%;; END OF THE FILE~%")
    ))