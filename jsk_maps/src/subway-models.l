(defun subway-table (&key name)
  (let (leg1 leg2 top table)
	(setq leg1
		  (body-
		   (make-prism (list (float-vector 100 450 0)
							 (float-vector 100 400 700)
							 (float-vector 100 -400 700)
							 (float-vector 100 -450 0))
					   (float-vector -200 0 0))
		   (make-prism (list (float-vector 100 350 0)
							 (float-vector 100 350 600)
							 (float-vector 100 -350 600)
							 (float-vector 100 -350 0))
					   (float-vector -200 0 0))))
	(setq leg2 (copy-object leg1))
	(send leg2 :rotate pi/2 :z)
	(setq top (make-cylinder 450 50))
	(send top :locate #f(0 0 690))
	(setq table (body+ leg1 leg2 top))
	(send table :set-color :white)
	(send table :name name)
	(send table :rotate-vertices (/ pi 4) :z)
	table
  ))

(defun subway-chair (&key (color :red) name)
  (let ((points (list (float-vector 0 0 0)
					  (float-vector 50 0 0)
					  (float-vector 180 0 180)
					  (float-vector 200 0 400)
					  (float-vector -200 0 400)
					  (float-vector -200 0 800)
					  (float-vector -210 0 800)
					  (float-vector -210 0 380)
					  (float-vector 160 0 360)
					  (float-vector 180 0 300)
					  (float-vector 0 0 0)
))
		chair)
	(setq chair (make-prism points (float-vector 0 300 0)))
	(send chair :translate-vertices (float-vector 0 -150 0))
;	(dotimes (i 5) (push (make-prism points #f(0 300 0)) blist))
;	(dotimes (i 5) (send (elt blist i) :translate (float-vector 0 (- 175 (* 65 i)) 0)))
;	blist
;	(apply #'body+ blist)
	(send chair :set-color color)
	(send chair :name name)
	chair
  ))

#|
    subway shop scene in eng2.bldg
    c is the origin of this scene, the end of the menu panel.

  [___________]     x ^
                      |
  @   c==   @     y<--o

         [table space]
|#

(defun make-subway-scene (&key ((:name n) "subway-shop"))
  (let (models menu pole1 pole2 store cashier walls table-set)
	;; manu panel (definition of origin)
	(setq menu (make-cube 100 1000 2000 :name "subway-menu-panel"))
	(send menu :set-color :gray)
	(send menu :translate (float-vector 50 -500 1000))
	(send menu :orient pi :z)
	;; poles
	(setq pole1 (make-cylinder 500 4000 :name "eng2-hall-pole"))
	(send pole1 :set-color :gray)
	(send pole1 :translate (float-vector 500 2000 0))
	(setq pole2 (make-cylinder 500 4000 :name "eng2-hall-pole"))
	(send pole2 :set-color :gray)
	(send pole2 :translate (float-vector 500 -3500 0))
	;; store and cashier
	(setq store (make-cube 800 4000 700 :name "subway-showcase"))
	(send store :set-color :green)
	(send store :translate (float-vector (+ 2000 (/ 800 2)) (- 1000 (/ 4000 2)) (/ 700 2)))
	(send store :orient pi :z)
	(setq cashier (make-cube 400 400 400 :name "subway-cashier"))
	(send cashier :set-color :yellow)
	(send cashier :translate (float-vector 2300 -2500 900))
	(send cashier :orient pi :z)
	;; tables and chairs
	(dotimes (i 10)
	  (setq table-set (list (subway-chair :color :red :name "subway-colorful-chair")
							(subway-chair :color :green :name "subway-colorful-chair")
							(subway-chair :color :blue :name "subway-colorful-chair")
							(subway-chair :color :yellow :name "subway-colorful-chair")
							(subway-table :name "subway-table")))
	  (dotimes (j 4)
		(send (elt table-set j) :orient (* (+ j i) pi/2) :z)
		(send (elt table-set j) :translate (float-vector -500 0 0)))
	  (send-all table-set :translate (float-vector (+ -3500 (* -3000 (/ i 5)))  (* -3000 (mod i 5)) 0) :world)
	  (setq models (append models table-set)))

	;; make scene instance
    (setq subway-shop
          (instance scene-model :init
                    :name n
                    :objects (flatten (list menu pole1 pole2
                              store cashier models
                              ))))
	;;
    subway-shop))

;(objects (list (make-subway-shop-scene)))
