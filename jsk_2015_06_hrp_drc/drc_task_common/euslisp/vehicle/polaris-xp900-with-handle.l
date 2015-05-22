#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/vehicle/drive-simulator-environment.l")

(defclass polaris-xp900-with-handle
  :super drive-simulator
  )

(defmethod polaris-xp900-with-handle
  (:init
   (&rest args)
   (send-super* :init args)
   (setq base nil
         handle nil
         pedal nil
         seat nil
         grasp-point nil)
   
   ;; make links links and assoc all links
   (setq base (send self :make-base-link))
   (setq seat (send self :make-seat-link))
   (setq handle (instance drive-sim-handle :init -140 325 -30 (/ 345 2) 400))
   (setq pedal (instance drive-sim-pedal :init))
   ;; (setq base (send self :make-drive-simulator-base))
   ;; (setq door (send self :make-drive-simulator-door))

   ;; move simulator
   (let ((pedal-x 380)
         (pedal-y 60)
         (pedal-z 350)
         (handle-x 530) ;; old -400
         (handle-y 480) ;; default
         (handle-z 840)
         (seat-x 0)
         (seat-y 0)
         (seat-z 740)
         )
     (send seat :translate (float-vector seat-x seat-y seat-z) :world)
     (send pedal :translate (float-vector pedal-x pedal-y pedal-z) :world)
     (send handle :translate (float-vector handle-x handle-y handle-z) :world)
     (send (send handle :handle-top) :rotate 20 :y :world)
     )

   ;; assoc links
   ;; Root link should be associated with "self".
   (send self :assoc base)
   (send base :assoc handle)
   (send base :assoc pedal)
   (send base :assoc seat)
   (setq links (list base handle pedal seat))

   ;; define slots for robot class
   ;; links and joint-list for cascaded-link.
   (send self :reset)
   
   ;; call :init-ending after defining links and joint-list and return "self"
   (send self :init-ending)
   self
   )
  (:make-base-link
   ()
   (let* (base-link
          car
          (floor-height 340)
          (footrest-height 110)
          handle-flame-list
          right-frame-center-grasp-point left-frame-center-grasp-point
          rear-frame-center-grasp-point rear-frame-left-grasp-point rear-frame-cross-left-grasp-point
          floor-grasp-point floor-footrest-grasp-point
          )
     ;; make handle-flame-list ;;;;;;;;;;;;;;;;;
     ;; make bodyset-link ;;;;;;;;;;;;;;;;;
     (setq car (with-open-file (f (format nil "~A/euslisp/vehicle/polaris-xp900.l" (ros::rospack-find "drc_task_common"))) (read f)))
     (setq base-link (instance bodyset-link :init (make-cascoords)
                               :bodies (list car)
                               :name :base-link))
     ;; define grasp point
     (setq right-frame-center-grasp-point (make-cascoords :coords (send (send base-link :copy-worldcoords)
                                                                   :translate (float-vector 50 -750 1700) :world)
                                                     :name :right-frame-center))
     (push right-frame-center-grasp-point grasp-point)
     (send car :assoc right-frame-center-grasp-point)
     (setq left-frame-center-grasp-point (make-cascoords :coords (send (send (send base-link :copy-worldcoords) :rotate (deg2rad -90) :y)
                                                                       :translate (float-vector 50 750 1700) :world)
                                                          :name :left-frame-center))
     (push left-frame-center-grasp-point grasp-point)
     (send car :assoc left-frame-center-grasp-point)
     (setq rear-frame-center-grasp-point (make-cascoords :coords (send (send (send (send base-link :copy-worldcoords) :rotate (deg2rad -90) :y) :rotate (deg2rad 90) :x)
                                                                       :translate (float-vector -350 0 1700) :world)
                                                          :name :rear-frame-center))
     (push rear-frame-center-grasp-point grasp-point)
     (send car :assoc rear-frame-center-grasp-point)
     (setq rear-frame-left-grasp-point (make-cascoords :coords (send (send rear-frame-center-grasp-point :copy-worldcoords) :translate #f(0 150 0) :world)
                                                       :name :rear-frame-left))
     (push rear-frame-left-grasp-point grasp-point)
     (send car :assoc rear-frame-left-grasp-point)
     (setq rear-frame-cross-left-grasp-point (make-cascoords :coords (send (send (send rear-frame-center-grasp-point :copy-worldcoords) :rotate (deg2rad 22.5) :x)
                                                                           :translate (float-vector 50 300 -380) :world)
                                                             :name :rear-frame-cross-left))
     (push rear-frame-cross-left-grasp-point grasp-point)
     (send car :assoc rear-frame-cross-left-grasp-point)
     (setq floor-grasp-point (make-cascoords :coords (send (send base-link :copy-worldcoords)
                                                                   :translate (float-vector 0 0 floor-height) :world)
                                                     :name :floor))
     (push floor-grasp-point grasp-point)
     (send car :assoc floor-grasp-point)
     (setq floor-footrest-grasp-point (make-cascoords :coords (send (send base-link :copy-worldcoords)
                                                                   :translate (float-vector 0 0 (+ floor-height footrest-height)) :world)
                                                     :name :floor-footrest))
     (push floor-footrest-grasp-point grasp-point)
     (send car :assoc floor-footrest-grasp-point)
     base-link
     )
   )
  (:make-seat-link
   (&optional (color :yellow))
   (let ((seat-width 1330)
         (seat-length 430)
         (seat-thickness 30)
         seat-list
         seat-link
         (rear-support-frame-height 240)
         ;; (rear-support-frame-y -292.5)
         ;; (rear-support-frame-y -150)
         (rear-support-frame-y 0) ;; temporary
         (rear-support-frame-x 50)
         )
     ;; make seat-list ;;;;;;;;;;;;;;;;;
     (let* ((seat-body (make-cube seat-length seat-width seat-thickness)))
       ;; move each body
       (send seat-body :translate (float-vector 0 0 (- (/ seat-thickness 2))))
       ;; set list and assoc
       (setq seat-list (list seat-body))
       ;; make bodyset-link
       (setq seat-link (instance bodyset-link :init (make-cascoords)
                                 :bodies (append seat-list nil)
                                 :name :seat-link))

       ;; define grasp point
       (setq seat-center-grasp-point (make-cascoords :coords (send (send seat-link :copy-worldcoords) :rotate (deg2rad 90) :y)
                                                     :name :seat-center))
       (push seat-center-grasp-point grasp-point)
       (send seat-body :assoc seat-center-grasp-point)
       (setq seat-left-grasp-point (make-cascoords :coords (send (send seat-center-grasp-point :copy-worldcoords) :translate #f(0 50 0) :world)
                                                   :name :seat-left))
       (push seat-left-grasp-point grasp-point)
       (send seat-body :assoc seat-left-grasp-point)
       (setq rear-support-frame-attachment-grasp-point (make-cascoords :coords (send (send (send seat-link :copy-worldcoords) :rotate (deg2rad 90) :y)
                                                                                     :translate (float-vector 0 rear-support-frame-y rear-support-frame-height) :world)
                                                                       :name :rear-support-frame-attachment))
       (push rear-support-frame-attachment-grasp-point grasp-point)
       (send seat-body :assoc rear-support-frame-attachment-grasp-point)

       (dolist (l (send seat-link :bodies))
         (send l :set-color color))
       seat-link
       )
     )
   )  
  (:reset
   (&optional (target nil))
   (case target
     (:handle (send handle :reset))
     (:pedals (send pedal :reset))
     (:accel (send pedal :reset :accel))
     (:brake (send pedal :reset :brake))
     (nil (send handle :reset)
          (send pedal :reset)
          )
     )
   )
  (:seat () seat)
  (:left-frame-center () (find-if #'(lambda (h) (eq (send h :name) :left-frame-center)) grasp-point))
  (:right-frame-center () (find-if #'(lambda (h) (eq (send h :name) :right-frame-center)) grasp-point))
  (:rear-frame-center () (find-if #'(lambda (h) (eq (send h :name) :rear-frame-center)) grasp-point))
  (:rear-frame-left () (find-if #'(lambda (h) (eq (send h :name) :rear-frame-left)) grasp-point))
  (:rear-frame-cross-left () (find-if #'(lambda (h) (eq (send h :name) :rear-frame-cross-left)) grasp-point))
  (:floor () (find-if #'(lambda (h) (eq (send h :name) :floor)) grasp-point))
  (:floor-footrest () (find-if #'(lambda (h) (eq (send h :name) :floor-footrest)) grasp-point))
  (:seat-center () (find-if #'(lambda (h) (eq (send h :name) :seat-center)) grasp-point))
  (:seat-left () (find-if #'(lambda (h) (eq (send h :name) :seat-left)) grasp-point))
  (:rear-support-frame-attachment () (find-if #'(lambda (h) (eq (send h :name) :rear-support-frame-attachment)) grasp-point))
  )

;; (setq hoge (instance polaris-xp900-with-handle :init))
;; (objects  (list hoge))
;; (send (send hoge :handle-crank) :draw-on :size 300 :flush t)
