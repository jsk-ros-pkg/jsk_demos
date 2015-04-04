;; origin of coords: center of front base frame
;; misumi frame chair with ranger xp900 size
(defclass misumi-ranger-xp900-chair
  :super cascaded-link
  :slots (handle reclining-joint mode)
  )

(defmethod misumi-ranger-xp900-chair
  (:init
   (&rest args &key (mode :single))
   (send-super* :init args)
   (setq seat nil)
   ;; 1. make links links and assoc all links
   (let ((seat (send self :make-seat-link mode))
         (back (send self :make-back-link mode)))
     (let ((back-pos-x -270) (back-pos-z 0))
       ;; 2. assoc links
       ;;    Root link should be associated with "self".
       (send back :translate (float-vector back-pos-x 0 back-pos-z) :world)
       (send self :assoc seat)
       (send seat :assoc back)
       ;; 3. make all joints
       ;;    Before making joints, you should :assoc all links.
       (setq reclining-joint (instance rotational-joint :init
                                       :parent-link seat :child-link back
                                       :name :reclining-joint :axis :y
                                       :min *-inf* :max *inf*))
       ;; 4. define slots for robot class
       ;;    links and joint-list for cascaded-link.
       (setq links (list seat back))
       (setq joint-list (list reclining-joint))
       (send self :reset)
       ;; 5. call :init-ending after defining links and joint-list and return "self"
       (send self :init-ending)
       self
       )
     )
   )
  
  (:reset
   ()
   (let ((reclining-joint-default-angle -20))
     (send self :reclining-joint :joint-angle reclining-joint-default-angle)
     )  
   )

  (:make-seat-link
   (mode)
   (case mode
     (:single (send self :make-seat-link-common 630))
     (:double (send self :make-seat-link-common 1300))
     )
   )

  (:make-back-link
   (mode)
   (case mode
     (:single (send self :make-back-link-common 630))
     (:double (send self :make-back-link-common 1300))
     )
   )   
  
  (:make-seat-link-common
   (arg-y-length)
   (let* ((pipe-thickness 40) (seat-leg-frame-length 350) (seat-x-frame-length 1000) (seat-y-frame-length arg-y-length)
          (plate-thickness 18) (plate-x-length 460)
          seat-x-frames seat-y-frames seat-leg-frames seat-plate
          seat-list seat-link
          )
     ;; make frames
     (dotimes (i 2) ;; 0: left 1: right
       (push (make-cube seat-x-frame-length pipe-thickness pipe-thickness) seat-x-frames)
       )
     (dotimes (i 2) ;; 0: front 1: back
       (push (make-cube pipe-thickness seat-y-frame-length pipe-thickness) seat-y-frames)
       )
     (dotimes (i 4) ;; 0: left-front 1: left-back 2: right-front 3: right-back
       (push (make-cube pipe-thickness pipe-thickness seat-leg-frame-length) seat-leg-frames)
       )
     (setq seat-plate (make-cube plate-x-length (+ seat-y-frame-length (* 2 pipe-thickness)) plate-thickness))

     ;; move frames
     (send (elt seat-x-frames 0) :translate (float-vector 0 (/ (+ seat-y-frame-length pipe-thickness) 2) 0))
     (send (elt seat-x-frames 1) :translate (float-vector 0 (- (/ (+ seat-y-frame-length pipe-thickness) 2)) 0))

     (send (elt seat-y-frames 0) :translate (float-vector (/ (- seat-x-frame-length pipe-thickness) 2) 0 0))
     (send (elt seat-y-frames 1) :translate (float-vector (+ (- (/ seat-x-frame-length 2) plate-x-length) (/ pipe-thickness 2)) 0 0))

     (send (elt seat-leg-frames 0) :translate (float-vector (/ (- seat-x-frame-length pipe-thickness) 2) (- (/ (+ seat-y-frame-length pipe-thickness) 2))
                                                            (- (/ (+ seat-leg-frame-length pipe-thickness) 2))))
     (send (elt seat-leg-frames 1) :translate (float-vector (- (/ (- seat-x-frame-length pipe-thickness) 2)) (- (/ (+ seat-y-frame-length pipe-thickness) 2))
                                                            (- (/ (+ seat-leg-frame-length pipe-thickness) 2))))
     (send (elt seat-leg-frames 2) :translate (float-vector (/ (- seat-x-frame-length pipe-thickness) 2) (/ (+ seat-y-frame-length pipe-thickness) 2)
                                                            (- (/ (+ seat-leg-frame-length pipe-thickness) 2))))
     (send (elt seat-leg-frames 3) :translate (float-vector (- (/ (- seat-x-frame-length pipe-thickness) 2)) (/ (+ seat-y-frame-length pipe-thickness) 2)
                                                            (- (/ (+ seat-leg-frame-length pipe-thickness) 2))))
     (send seat-plate :translate (float-vector (/ (- seat-x-frame-length plate-x-length) 2) 0
                                               (/ (+ pipe-thickness plate-thickness) 2)))
     
     ;; set list and assoc
     (setq seat-list (append seat-x-frames seat-y-frames seat-leg-frames))
     (push seat-plate seat-list)
     (dolist (b (cdr seat-list)) (send (car seat-list) :assoc b))
     (send (car seat-list) :translate (float-vector (- (/ (- seat-x-frame-length plate-x-length) 2)) 0 (- (+ (/ pipe-thickness 2) plate-thickness)))) ;; move origin to center of seat
     
     ;; set bodyset seat-link
     (setq seat-link (instance bodyset-link :init (make-cascoords)
                               :bodies (append seat-list nil)
                               :name :seat-link))
     ;; set color
     (dolist (l (cdr (send seat-link :bodies)))
       (send l :set-color :gray60))
     (send (car (send seat-link :bodies)) :set-color :yellow)    
     ;; (objects seat-link)
     seat-link
     )
   )
  
  (:make-back-link-common
   (arg-y-length)
   (let* ((pipe-thickness 40) (back-z-frame-length 530) (back-y-frame-length arg-y-length)
          (plate-thickness 18) (plate-z-length 460)
          back-z-frames back-y-frames back-plate
          back-list back-link
          )
     ;; make frames
     (dotimes (i 2) ;; 0: left 1: right
       (push (make-cube pipe-thickness pipe-thickness back-z-frame-length) back-z-frames)
       )
     (dotimes (i 2) ;; 0: front 1: back
       (push (make-cube pipe-thickness back-y-frame-length pipe-thickness) back-y-frames)
       )
     (setq back-plate (make-cube plate-thickness (+ back-y-frame-length (* 2 pipe-thickness)) plate-z-length))

     ;; move frames
     (send (elt back-z-frames 0) :translate (float-vector 0 (/ (+ back-y-frame-length pipe-thickness) 2) 0))
     (send (elt back-z-frames 1) :translate (float-vector 0 (- (/ (+ back-y-frame-length pipe-thickness) 2)) 0))

     (send (elt back-y-frames 0) :translate (float-vector 0 0 (/ (- back-z-frame-length pipe-thickness) 2)))
     (send (elt back-y-frames 1) :translate (float-vector 0 0 (+ (- (/ back-z-frame-length 2) plate-z-length) (/ pipe-thickness 2))))

     (send back-plate :translate (float-vector (/ (+ pipe-thickness plate-thickness) 2) 0 (/ (- back-z-frame-length plate-z-length) 2)))
     
     ;; set list and assoc
     (setq back-list (append back-z-frames back-y-frames))
     (push back-plate back-list)
     (dolist (b (cdr back-list)) (send (car back-list) :assoc b))
     (send (car back-list) :translate (float-vector 0 0 (/ back-z-frame-length 2))) ;; move origin to root of back

     ;; set bodyset seat-link
     (setq back-link (instance bodyset-link :init (make-cascoords)
                               :bodies (append back-list nil)
                               :name :back-link))
     ;; set color
     (dolist (l (cdr (send back-link :bodies)))
       (send l :set-color :gray60))
     (send (car (send back-link :bodies)) :set-color :yellow)    
     ;; (objects back-link)
     back-link
     )
   )
  ;; methods to get handle ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  ;; (:seat () (find-if #'(lambda (h) (eq (send h :name) :seat)) handle))
  ;; (:seat-left () (find-if #'(lambda (h) (eq (send h :name) :seat-left)) handle))
  ;; (:seat-right () (find-if #'(lambda (h) (eq (send h :name) :seat-right)) handle))
  ;; (:back-left () (find-if #'(lambda (h) (eq (send h :name) :back-left)) handle))
  ;; (:back-right () (find-if #'(lambda (h) (eq (send h :name) :back-right)) handle)) 
  ;; methods to send message to joitns ;;;;;;;;;;;;;;;;; ;;;;;;;;;;;;;;;;;  
  (:reclining-joint (&rest args) (forward-message-to reclining-joint args))
  )
