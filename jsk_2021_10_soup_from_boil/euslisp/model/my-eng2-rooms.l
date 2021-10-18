;; (+ 2707 2707 2707) -> 8121
;; (+ 2707 2707 2707 2707 525) -> 11353
;; (+ 4050 3450 3450 3600) -> 14550
;; 2202
;(load "models/room73b1-scene.l")
;; (load "models/room73b2-scene.l")
(load "models/room73b2-scene.l")
(load "package://jsk_2021_10_soup_from_boil/euslisp/model/my-room73b2-scene.l")
(load "models/room73a3-scene.l")
(load "models/room83b1-scene.l")
;; (load "subway-models.l")
(load "package://jsk_maps/src/subway-models.l")
;;
(defmethod scene-model
  (:set-color (c) (dolist (it objs)
                    (when (find-method it :set-color)
                      (send-all objs :set-color c)))))
(defun make-eng2-room (w1 w2 d &rest args &key (name) &allow-other-keys)
  (let* (b (s 50) (sd (if (> d 0) (- s) s)) k v room)
    ;; w1 > 0, w2 < 0
    (setq b (make-prism (list (float-vector sd s 0)
                  (float-vector sd (- w1 s) 0)
                  (float-vector (- 0 d sd) (- w1 s) 0)
                  (float-vector (- 0 d sd) (- s w2) 0)
                  (float-vector sd (- s w2) 0)
                  (float-vector sd s 0)
                  )
            1.0 :name name))
    (setq room (instance scene-model :init :name name :objects (list b)))
    (send room :assoc b)
    (while args
      (setq k (pop args) v (pop args))
      (setf (get room k) v))
    room))

(defun make-eng2-corridor (w l &key (pos (float-vector 0 0 0)) (rpy (float-vector 0 0 0)) (name "corridor"))
  (let (b)
    (setq b (make-prism (list (float-vector 0 0 0)
                  (float-vector 0 (/ w 2) 0)
                  (float-vector l (/ w 2) 0)
                  (float-vector l (/ w -2) 0)
                  (float-vector 0 (/ w -2) 0)
                  (float-vector 0 0 0))
            1 :name name))
    (send b :locate pos)
    (send b :rpy (elt rpy 0) (elt rpy 1) (elt rpy 2))
    (setf (get b :face-color) :orange)
    b))


(defun make-71-rooms ()
  (let ((a1 (make-eng2-room 0 (+ (* 1150 3) 600 250) 8121 :name "71a1"
            :knowrob-type "ProfessorsOffice"))
    (a2 (make-eng2-room 0 (* 1150 3) 8121 :name "71a2"
            :knowrob-type "ProfessorsOffice"))
    (a3 (make-eng2-room 0 (* 1150 3) 8121 :name "71a3"
            :knowrob-type "ProfessorsOffice"))
    (a4 (make-eng2-room 150 (* 1150 3) 8121 :name "71a4"
            :knowrob-type "ProfessorsOffice"))
    (b2 (make-eng2-room (+ (* 1150 4) 900) (+ (* 1150 3) 100 500 295) -11353 :name "71b"
            :knowrob-type "LaboratoryRoom"))
    (c1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 100 500 295) 11353 :name "71c1"
            :knowrob-type "LaboratoryRoom"))
    (c2 (make-eng2-room (+ (* 1150 3) 100) (* 1150 3) 11353 :name "71c2"
            :knowrob-type "LaboratoryRoom"))
    (d1 (make-eng2-room (* 1150 3) (+ 100 500 295) -8121 :name "71d1"
            :knowrob-type "ProfessorsOffice"))
    (d2 (make-eng2-room (* 1150 3) 0 -8121 :name "71d2"
            :knowrob-type "LaboratoryRoom"))
    (d3 (make-eng2-room (* 1150 3) 0 -8121 :name "71d3"
            :knowrob-type "ProfessorsOffice"))
    (d4 (make-eng2-room (+ (* 1150 3) 100) 0 -8121 :name "71d4"
            :knowrob-type "StorageRoom"))
    )
    (setf (get b2 :face-color) :dodgerblue)
    (setf (get c2 :face-color) :dodgerblue)
    (setf (get a1 :face-color) :dodgerblue)
    (send d4 :locate (float-vector 0 (* 3 1150) 0))
    (send d3 :assoc d4)
    (send d3 :locate (float-vector 0 (* 3 1150) 0))
    (send d2 :assoc d3)
    (send d2 :locate (float-vector 0 (* 3 1150) 0))
    (send d1 :assoc d2)
    (send c2 :locate (float-vector 0 (* 6 1150) 0))
    (send c1 :assoc c2)
    (send d1 :locate (float-vector 0 (* -3 1150) 0))
    (send c1 :locate (float-vector -2020 0 0))
    (send c1 :assoc d1)

    (send a4 :locate (float-vector 0 (* 3 1150) 0))
    (send a3 :assoc a4)
    (send a3 :locate (float-vector 0 (* 3 1150) 0))
    (send a2 :assoc a3)
    (send a2 :locate (float-vector 0 (* 3 1150) 0))
    (send a1 :assoc a2)
    ;;
    (send b2 :locate (float-vector 2020 0 0))
    (send a1 :assoc b2)
    (send a1 :locate (float-vector 0 (- (+ (* 9 1150) 150)) 0))

    (send a1 :locate (float-vector -1010 0 0))
    ;;
    (send c1 :locate (float-vector 0 (* -9 1150) 0))
    ;(send a1 :locate (float-vector 0 -1010 0))
    (send c1 :locate (float-vector 1010 0 0))
    (send c1 :locate (float-vector (+ 33319 2020) 0 0))
    (send a1 :assoc c1)
    (list a1 a2 a3 a4 b2 c1 c2 d1 d2 d3 d4)))

(defun make-72-rooms ()
  (let ((a1 (make-eng2-room 0 (+ (* 1150 3) 150) 8121 :name "72a1"
            :knowrob-type "ProfessorsOffice"))
    (a2 (make-eng2-room 0 (* 1150 3) 8121 :name "72a2"
            :knowrob-type "LaboratoryRoom"))
    (a3 (make-eng2-room 0 (* 1150 3) 8121 :name "72a3"
            :knowrob-type "LaboratoryRoom"))
    (a4 (make-eng2-room 150 (+ (* 1150 3)) 8121 :name "72a4"
            :knowrob-type "ProfessorsOffice"))
    (b1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 150) -11353 :name "72b1"
            :knowrob-type "LaboratoryRoom"))
    (b2 (make-eng2-room (+ (* 1150 3) 150) (* 1150 3) -11353 :name "72b2"
            :knowrob-type "LaboratoryRoom"))
    ;;
    (c1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 150) 11353 :name "72c1"
            :knowrob-type "LaboratoryRoom"))
    (c2 (make-eng2-room (+ (* 1150 3) 150) (* 1150 3) 11353 :name "72c2"
            :knowrob-type "LaboratoryRoom"))
    (d1 (make-eng2-room (* 1150 3) 150 -8121 :name "72d1"
            :knowrob-type "ProfessorsOffice"))
    (d2 (make-eng2-room (* 1150 3) 0 -8121 :name "72d2"
            :knowrob-type "ProfessorsOffice"))
    (d3 (make-eng2-room (* 1150 3) 0 -8121 :name "72d3"
            :knowrob-type "LaboratoryRoom"))
    (d4 (make-eng2-room (+ (* 1150 3) 150) 0 -8121 :name "72d4"
            :knowrob-type "LaboratoryRoom")))
    (setf (get b1 :face-color) :dodgerblue)
    (setf (get b2 :face-color) :dodgerblue)
    (setf (get d1 :face-color) :dodgerblue)
    (setf (get a1 :face-color) :dodgerblue)

    (send d4 :locate (float-vector 0 (* 3 1150) 0))
    (send d3 :assoc d4)
    (send d3 :locate (float-vector 0 (* 3 1150) 0))
    (send d2 :assoc d3)
    (send d2 :locate (float-vector 0 (* 3 1150) 0))
    (send d1 :assoc d2)
    (send d1 :locate (float-vector 0 150 0))
    ;;
    (send c2 :locate (float-vector 0 (* 6 1150) 0))
    (send c1 :assoc c2)
    (send c1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send c1 :locate (float-vector -2020 0 0))
    (send c1 :assoc d1)

    (send b2 :locate (float-vector 0 (* 6 1150) 0))
    (send b1 :assoc b2)
    (send a4 :locate (float-vector 0 (* 3 1150) 0))
    (send a3 :assoc a4)
    (send a3 :locate (float-vector 0 (* 3 1150) 0))
    (send a2 :assoc a3)
    (send a2 :locate (float-vector 0 (* 3 1150) 0))
    (send a1 :assoc a2)
    (send a1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    ;;
    (send b1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send b1 :locate (float-vector 2020 0 0))
    (send a1 :assoc b1)

    (send a1 :locate (float-vector -1010 0 0))
    (send c1 :locate (float-vector 1010 0 0))
    (send c1 :locate (float-vector (+ 33319 2020) 0 0))
    (send a1 :assoc c1)

    (list a1 a2 a3 a4 b1 b2 c1 c2 d1 d2 d3 d4)))

(defun make-73-rooms ()
  (let ((a1 (make-eng2-room 0 (+ (* 1150 3) 150) 8121 :name "73a1"
            :people "Masayuki Inaba" :knowrob-type "ProfessorsOffice"))
        (a2 (make-eng2-room 0 (* 1150 3) 8121 :name "73a2"
            :people "kei Okada" :knowrob-type "ProfessorsOffice"))
        (a3 (let ((a3 (instance room73a3-scene :init :name "73a3")))
            (setf (get a3 :knowrob-type) "OfficeRoom") a3))
        (a4 (make-eng2-room (+ 100 500 295) (* 1150 3) 8121 :name "73a4"
            :knowrob-type "LaboratoryRoom"))
        (b1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 150) -11353 :name "73b1"
            :knowrob-type "LaboratoryRoom"))
        (b2 (let ((b2 (instance room73b2-scene :init :name "73b2")))
            (setf (get b2 :knowrob-type) (list "Kitchen" "LaboratoryRoom")) b2))
        (c1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 150) 11353 :name "73c1"
            :knowrob-type "OfficeRoom"))
        (c2 (make-eng2-room (+ (* 1150 3) 100 500 295) (* 1150 3) 11353 :name "73c2"
            :knowrob-type "MeetingSpace"))
        (d1 (make-eng2-room (* 1150 3) 150 -8121 :name "73d1"
            :knowrob-type "LaboratoryRoom"))
        (d2 (make-eng2-room (* 1150 3) 0 -8121 :name "73d2"
            :knowrob-type "LaboratoryRoom"))
        (d3 (make-eng2-room (* 1150 3) 0 -8121 :name "73d3"
            :knowrob-type "ProfessorsOffice"))
        (d4 (make-eng2-room (+ (* 1150 3) 100 500 295) 0 -8121 :name "73d4"
            :knowrob-type "ProfessorsOffice"))
        (7a-mini-kitchen
            (make-eng2-room (+ 850 400) 0 -2000 :name "7a-mini-kitchen"
            :knowrob-type (list "Kitchen" "UtilityRoom")))
        )
    (send a1 :set-color :deeppink)
    (send a2 :set-color :lightpink)
    (send a3 :set-color :lawngreen)
    (send a4 :set-color :dodgerblue)
    (send b1 :set-color :darkviolet)
    (send b2 :set-color :dodgerblue)
    (send d1 :set-color :dodgerblue)
    (send 7a-mini-kitchen :set-color :blue)
    (dolist (obj (send b2 :objects)) (send b2 :assoc obj))
    (dolist (obj (send a3 :objects)) (send a3 :assoc obj))

    (send d4 :locate (float-vector 0 (* 3 1150) 0))
    (send d3 :assoc d4)
    (send d3 :locate (float-vector 0 (* 3 1150) 0))
    (send d2 :assoc d3)
    (send d2 :locate (float-vector 0 (* 3 1150) 0))
    (send d1 :assoc d2)
    (send d1 :locate (float-vector 0 150 0))
    ;;
    (send c2 :locate (float-vector 0 (* 6 1150) 0))
    (send c1 :assoc c2)
    (send c1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send c1 :locate (float-vector -2020 0 0))
    (send c1 :assoc d1)

    ;; (send b2 :locate (float-vector 0 (* 6 1150) 0))
    (send b2 :locate (float-vector 0 (- (* 6 1150) 300) 0))
    (send b1 :assoc b2)
    (send b1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send a4 :locate (float-vector 0 (* 3 1150) 0))
    (send a3 :assoc a4)
    (send a3 :locate (float-vector 0 (* 3 1150) 0))
    (send a2 :assoc a3)
    (send a2 :locate (float-vector 0 (* 3 1150) 0))
    (send a1 :assoc a2)
    (send 7a-mini-kitchen :locate
      (float-vector -300 (- (+ (* 3 1150) 2000 250)) 0))
    (send 7a-mini-kitchen :rotate pi/2 :z)
    (send a1 :assoc 7a-mini-kitchen)
    (send a1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))

    (send b1 :locate (float-vector 2020 0 0))
    (send a1 :assoc b1)

    (send a1 :locate (float-vector -1010 0 0))
    (send c1 :locate (float-vector 1010 0 0))
    (send c1 :locate (float-vector (+ 33319 2020) 0 0))
    (send a1 :assoc c1)

    ;;
    (send a3 :dissoc a4)
    (send a3 :orient pi :z)

    ;;
    (list a1 a2 a3 a4 b1 b2 c1 c2 d1 d2 d3 d4 7a-mini-kitchen)))



(defun make-81-rooms ()
  (let ((a1 (make-eng2-room 0 (+ (* 1150 3) 600 250) 8121 :name "81a1"))
    (a2 (make-eng2-room 0 (* 1150 3) 8121 :name "81a2"))
    (a3 (make-eng2-room 0 (* 1150 3) 8121 :name "81a3"))
    (a4 (make-eng2-room 150 (* 1150 3) 8121 :name "81a4"))
    (b2 (make-eng2-room (+ (* 1150 4) 900) (+ (* 1150 3) 100 500 295) -11353 :name "81b2"))
    (c1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 100 500 295) 11353 :name "81c1"))
    (c2 (make-eng2-room (+ (* 1150 3) 100) (* 1150 3) 11353 :name "81c2"))
    (d1 (make-eng2-room (* 1150 3) (+ 100 500 295) -8121 :name "81d1"))
    (d2 (make-eng2-room (* 1150 3) 0 -8121 :name "81d2"))
    (d3 (make-eng2-room (* 1150 3) 0 -8121 :name "81d3"))
    (d4 (make-eng2-room (+ (* 1150 3) 100) 0 -8121 :name "81d4"))
    )
    (setf (get b2 :face-color) :dodgerblue)
    (setf (get c2 :face-color) :dodgerblue)
    (setf (get a1 :face-color) :dodgerblue)
    (send d4 :locate (float-vector 0 (* 3 1150) 0))
    (send d3 :assoc d4)
    (send d3 :locate (float-vector 0 (* 3 1150) 0))
    (send d2 :assoc d3)
    (send d2 :locate (float-vector 0 (* 3 1150) 0))
    (send d1 :assoc d2)
    (send c2 :locate (float-vector 0 (* 6 1150) 0))
    (send c1 :assoc c2)
    (send d1 :locate (float-vector 0 (* -3 1150) 0))
    (send c1 :locate (float-vector -2020 0 0))
    (send c1 :assoc d1)

    (send a4 :locate (float-vector 0 (* 3 1150) 0))
    (send a3 :assoc a4)
    (send a3 :locate (float-vector 0 (* 3 1150) 0))
    (send a2 :assoc a3)
    (send a2 :locate (float-vector 0 (* 3 1150) 0))
    (send a1 :assoc a2)
    ;;
    (send b2 :locate (float-vector 2020 0 0))
    (send a1 :assoc b2)
    (send a1 :locate (float-vector 0 (- (+ (* 9 1150) 150)) 0))

    (send a1 :locate (float-vector -1010 0 0))
    ;;
    (send c1 :locate (float-vector 0 (* -9 1150) 0))
    ;(send a1 :locate (float-vector 0 -1010 0))
    (send c1 :locate (float-vector 1010 0 0))
    (send c1 :locate (float-vector (+ 33319 2020) 0 0))
    (send a1 :assoc c1)
    (list a1 a2 a3 a4 b2 c1 c2 d1 d2 d3 d4)))

(defun make-82-rooms ()
  (let ((a1 (make-eng2-room 0 (+ (* 1150 3) 150) 8121 :name "82a1"))
    (a2 (make-eng2-room 0 (* 1150 3) 8121 :name "82a2"))
    (a3 (make-eng2-room 0 (* 1150 3) 8121 :name "82a3"))
    (a4 (make-eng2-room 150 (+ (* 1150 3)) 8121 :name "82a4"))
    (b1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 150) -11353 :name "82b1"))
    (b2 (make-eng2-room (+ (* 1150 3) 150) (* 1150 3) -11353 :name "82b2"))
    ;;
    (c1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 150) 11353 :name "82c1"))
    (c2 (make-eng2-room (+ (* 1150 3) 150) (* 1150 3) 11353 :name "82c2"))
    (d1 (make-eng2-room (* 1150 3) 150 -8121 :name "82d1"))
    (d2 (make-eng2-room (* 1150 3) 0 -8121 :name "82d2"))
    (d3 (make-eng2-room (* 1150 3) 0 -8121 :name "82d3"))
    (d4 (make-eng2-room (+ (* 1150 3) 150) 0 -8121 :name "82d4")))
    (setf (get b1 :face-color) :dodgerblue)
    (setf (get b2 :face-color) :dodgerblue)
    (setf (get d1 :face-color) :dodgerblue)
    (setf (get a1 :face-color) :dodgerblue)

    (send d4 :locate (float-vector 0 (* 3 1150) 0))
    (send d3 :assoc d4)
    (send d3 :locate (float-vector 0 (* 3 1150) 0))
    (send d2 :assoc d3)
    (send d2 :locate (float-vector 0 (* 3 1150) 0))
    (send d1 :assoc d2)
    (send d1 :locate (float-vector 0 150 0))
    ;;
    (send c2 :locate (float-vector 0 (* 6 1150) 0))
    (send c1 :assoc c2)
    (send c1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send c1 :locate (float-vector -2020 0 0))
    (send c1 :assoc d1)

    (send b2 :locate (float-vector 0 (* 6 1150) 0))
    (send b1 :assoc b2)
    (send a4 :locate (float-vector 0 (* 3 1150) 0))
    (send a3 :assoc a4)
    (send a3 :locate (float-vector 0 (* 3 1150) 0))
    (send a2 :assoc a3)
    (send a2 :locate (float-vector 0 (* 3 1150) 0))
    (send a1 :assoc a2)
    (send a1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    ;;
    (send b1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send b1 :locate (float-vector 2020 0 0))
    (send a1 :assoc b1)

    (send a1 :locate (float-vector -1010 0 0))
    (send c1 :locate (float-vector 1010 0 0))
    (send c1 :locate (float-vector (+ 33319 2020) 0 0))
    (send a1 :assoc c1)

    (list a1 a2 a3 a4 b1 b2 c1 c2 d1 d2 d3 d4)))

(defun make-83-rooms ()
  (let ((a1 (make-eng2-room 0 (+ (* 1150 3) 150) 8121 :name "83a1"))
    (a2 (make-eng2-room 0 (* 1150 3) 8121 :name "83a2"
            :knowrob-type '("ServerRoom" "StorageRoom" "Workshop")))
    (a3 (make-eng2-room 0 (* 1150 3) 8121 :name "83a3"))
    (a4 (make-eng2-room (+ 100 500 295) (* 1150 3) 8121 :name "83a4"))
    (b1 (let ((b1 (instance room83b1-scene :init :name "83b1"))) (setf (get b1 :knowrob-type) "LaboratoryRoom") b1))
    (b2 (make-eng2-room (+ (* 1150 3) 100 500 295) (* 1150 3) -11353 :name "83b2"
            :knowrob-type "LaboratoryRoom"))
    (c1 (make-eng2-room (* 1150 3) (+ (* 1150 3) 150) 11353 :name "83c1"))
    (c2 (make-eng2-room (+ (* 1150 3) 100 500 295) (* 1150 3) 11353 :name "83c2"))
    (d1 (make-eng2-room (* 1150 3) 150 -8121 :name "83d1"))
    (d2 (make-eng2-room (* 1150 3) 0 -8121 :name "83d2"))
    (d3 (make-eng2-room (* 1150 3) 0 -8121 :name "83d3"))
    (d4 (make-eng2-room (+ (* 1150 3) 100 500 295) 0 -8121 :name "83d4"))
        (8a-mini-kitchen
            (make-eng2-room (+ 850 400) 0 -2000 :name "8a-mini-kitchen"
            :knowrob-type (list "Kitchen" "UtilityRoom")))
        )
    (setf (get a1 :face-color) :deeppink)
    (setf (get a2 :face-color) :lightpink)
    (setf (get a3 :face-color) :lawngreen)
    (setf (get a4 :face-color) :dodgerblue)
    (setf (get b1 :face-color) :darkviolet)
    (setf (get b2 :face-color) :dodgerblue)
    (setf (get d1 :face-color) :dodgerblue)
    (dolist (obj (send b1 :objects)) (send b1 :assoc obj))

    (send d4 :locate (float-vector 0 (* 3 1150) 0))
    (send d3 :assoc d4)
    (send d3 :locate (float-vector 0 (* 3 1150) 0))
    (send d2 :assoc d3)
    (send d2 :locate (float-vector 0 (* 3 1150) 0))
    (send d1 :assoc d2)
    (send d1 :locate (float-vector 0 150 0))
    ;;
    (send c2 :locate (float-vector 0 (* 6 1150) 0))
    (send c1 :assoc c2)
    (send c1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send c1 :locate (float-vector -2020 0 0))
    (send c1 :assoc d1)

    (send b2 :locate (float-vector 0 (* 6 1150) 0))
    (send b1 :assoc b2)
    (send b1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))
    (send a4 :locate (float-vector 0 (* 3 1150) 0))
    (send a3 :assoc a4)
    (send a3 :locate (float-vector 0 (* 3 1150) 0))
    (send a2 :assoc a3)
    (send a2 :locate (float-vector 0 (* 3 1150) 0))
    (send a1 :assoc a2)
    (send 8a-mini-kitchen :locate
      (float-vector -300 (- (+ (* 3 1150) 2000 250)) 0))
    (send 8a-mini-kitchen :rotate pi/2 :z)
    (send a1 :assoc 8a-mini-kitchen)
    (send a1 :locate (float-vector 0 (+ (* 3 1150) 150) 0))

    (send b1 :locate (float-vector 2020 0 0))
    (send a1 :assoc b1)

    (send a1 :locate (float-vector -1010 0 0))
    (send c1 :locate (float-vector 1010 0 0))
    (send c1 :locate (float-vector (+ 33319 2020) 0 0))
    (send a1 :assoc c1)
    ;;
    (list a1 a2 a3 a4 b1 b2 c1 c2 d1 d2 d3 d4 8a-mini-kitchen)))

(defun eng2-2f-rooms ()
  (let ((21a  (make-eng2-room 0 (* 1150 6) 6500 :name "21a" :knowrob-type "LockerRoom"))
        (21b2 (make-eng2-room 0 (* 1150 3) 9000 :name "21b2" :knowrob-type "ControlRoom"))
        (22a  (make-eng2-room (* 1150 13) 0 18000 :name "22a" :knowrob-type "LectureHall"))
        (22b  (make-eng2-room (* 1150 8) 0 18000 :name "22b" :knowrob-type "LaboratoryRoom"))
;       (23a  (make-eng2-room 0 (* 1150 3) 8121 :name "23a" :knowrob-type ""))
;       (23b  (make-eng2-room 0 (* 1150 3) 8121 :name "23b" :knowrob-type ""))
        (subway (make-subway-scene :name "subway-shop")))
    (send 21a :locate (float-vector -7500 -39200 0))
    (send 21a  :orient pi/2 :z)
    (send 21b2  :locate (float-vector 8700 -35500 0))
    (send 21b2  :orient pi/2 :z)
    (send 22a :set-color :red)
    (send 22a :locate (float-vector -2800 -25500 0))
    (send 22a  :orient (- pi/2) :z)
    (send 22b :locate (float-vector -12100 -25500 0))
    (send 22b  :orient (- pi/2) :z)
    (setf (get subway :knowrob-type) "FastFoodRestaurantSpace")
    (dolist (obj (send subway :objects)) (send subway :assoc obj))
    (send subway :locate (float-vector 23300 -8200 0))
    (send subway :orient pi/2 :z)
    ;;
    (list 21a 21b2 22a 22b subway)
    ))

(defun eng2-7f-rooms ()
  (let (71rooms 72rooms 73rooms corridors 73b1 rooms)
    (setq 71rooms (make-71-rooms)
          72rooms (make-72-rooms)
          73rooms (make-73-rooms))
    (send (car 72rooms) :locate (float-vector 0 (- (+ 1500 445 2710 445 1500   3600 3450 3450 3600)) 0))
    (send (car 71rooms) :locate (float-vector 0 (- (+ 1500 445 2710 445 1500   3600 3450 3450 3600   1500 445 2710 445 1500)) 0))

    (setq corridors
          (list (make-eng2-corridor 2000 14800 :pos (float-vector 0 0 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14100 :pos (float-vector 0 (- (+ 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14800 :pos (float-vector 0 (- (+ 1500 445 2710 445 1500  14100 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2700 33319 :pos (float-vector 1010 (- (+ 1500 445 (/ 2710 2))) 0))
                (make-eng2-corridor 2000 14800 :pos (float-vector (+ 2020 33319) 0 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14100 :pos (float-vector (+ 2020 33319) (- (+ 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14800 :pos (float-vector (+ 2020 33319) (- (+ 1500 445 2710 445 1500  14100 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2700 33319 :pos (float-vector 1010 (- (+ 14800 (* 2 (+ 2710 1500 445)))) 0))
                ))
    (let ((id 0))
      (dolist (co corridors)
        (send co :name (format nil "corridor~a" (incf id)))))

    (setq rooms (append 71rooms 72rooms 73rooms corridors))
    (setq 73b1 (find "73b1" rooms
                         :test #'string= :key #'(lambda(x)(send x :name))))
    (dolist (rm rooms)
      (when (send rm :parent) (send (send rm :parent) :dissoc rm))
      (gl::transparent rm 0.3))

    rooms ))

(defun eng2-8f-rooms ()
  (let (81rooms 82rooms 83rooms corridors 83b1 rooms)
    (setq 81rooms (make-81-rooms)
          82rooms (make-82-rooms)
          83rooms (make-83-rooms))
    (send (car 82rooms) :locate (float-vector 0 (- (+ 1500 445 2710 445 1500   3600 3450 3450 3600)) 0))
    (send (car 81rooms) :locate (float-vector 0 (- (+ 1500 445 2710 445 1500   3600 3450 3450 3600   1500 445 2710 445 1500)) 0))

    (setq corridors
          (list (make-eng2-corridor 2000 14800 :pos (float-vector 0 0 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14100 :pos (float-vector 0 (- (+ 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14800 :pos (float-vector 0 (- (+ 1500 445 2710 445 1500  14100 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2700 33319 :pos (float-vector 1010 (- (+ 1500 445 (/ 2710 2))) 0))
                (make-eng2-corridor 2000 14800 :pos (float-vector (+ 2020 33319) 0 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14100 :pos (float-vector (+ 2020 33319) (- (+ 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2000 -14800 :pos (float-vector (+ 2020 33319) (- (+ 1500 445 2710 445 1500  14100 1500 445 2710 445 1500)) 0) :rpy (float-vector pi/2 0 0))
                (make-eng2-corridor 2700 33319 :pos (float-vector 1010 (- (+ 14800 (* 2 (+ 2710 1500 445)))) 0))
                ))
    (let ((id 0))
      (dolist (co corridors)
        (send co :name (format nil "corridor~a" (incf id)))))

    (setq rooms (append 81rooms 82rooms 83rooms))
    (setq 83b1 (find "83b1" rooms
                     :test #'string= :key #'(lambda(x)(send x :name))))
    (dolist (rm rooms)
      (when (send rm :parent) (send (send rm :parent) :dissoc rm))
      (gl::transparent rm 0.3))

    rooms ))

;;(objects rooms)
;; (dolist (r rooms)
;;   (format t "~A:~%" (get r :name))
;;   (format t "  pos -> ~A~%" (send r :pos))
;;   (format t "  rot -> ~A~%" (send r :rpy-angle))
;;   (if (get r :people)
;;       (format t "  people -> ~A~%" (get r :people))))
