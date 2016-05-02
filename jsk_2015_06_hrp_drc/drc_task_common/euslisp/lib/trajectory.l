;; trajectory.l
(ros::roseus-add-msgs "visualization_msgs")

(defun range (max)
  (let ((ret nil))
    (dotimes (i max)
      (setq ret (append ret (list i)))
      )
    ret))

(defun coords-set-to-arrow (coords-set)
  (require "models/arrow-object")
  (mapcar #'(lambda (c)
              (instance arrow-object
                        :init
                        :pos (send c :worldpos)
                        :rot (send c :worldrot)))
              coords-set))

(defun animate-coords-set-as-arrow (coords-set)
  (require "models/arrow-object")
  (let ((original-objects (send *irtviewer* :objects)))
    (dolist (c coords-set)
      (let ((arrow (instance arrow-object
                             :init
                             :pos (send c :worldpos)
                             :rot (send c :worldrot))))
        (send *irtviewer* :objects (cons arrow original-objects))
        (send *irtviewer* :draw-objects)
        ))
    (send *irtviewer* :objects original-objects)
    ))

(defun max-index (lst)
  (let ((max-index 0)
        (max-value (car lst)))
    (dotimes (i (length (cdr lst)))
      (let ((v (elt lst (1+ i))))
        (if (> v max-value)
            (progn
              (setq max-value v)
              (setq max-index (1+ i))))
      ))
    max-index))

(defun coords-distance (a b)
  ;; 1deg = 1mm
  (+ (norm (send a :difference-position b))
     (rad2deg (norm (send a :difference-rotation b)))))

(defclass manipulation-trajectory
  :super propertied-object
  :slots (min-via-coords-points via-coords parent-object))

(defmethod manipulation-trajectory
  (:init (&key ((:min-via-coords-points amin-via-coords))
               ((:via-coords avia-coords))
               ((:parent-object aparent-object)))
    (if (null amin-via-coords)
        (setq amin-via-coords (range (length avia-coords))))
    (setq min-via-coords-points amin-via-coords)
    (setq via-coords avia-coords)
    (setq parent-object aparent-object)
    (dolist (c via-coords)
      (send parent-object :assoc c))
    (setf (get self :line-width) 1.0)
    self)
  (:min-via-coords-points ()
    min-via-coords-points)
  (:min-resolution ()
    (length min-via-coords-points))
  (:via-coords (&optional (resolution-num 0))
    (if (< resolution-num (send self :min-resolution)) ;overwrite resolution-num
        (setq resolution-num (send self :min-resolution)))
    ;; first extract min coords
    (let ((min-via-coords (mapcar #'(lambda (p)
                                      (send self :interpolate-raw p))
                                  min-via-coords-points)))
      (let ((rest-resolution (- resolution-num (length min-via-coords)))
            (via-coords min-via-coords))
        ;; 1. compute distance between coords
        ;; 2. select the longest coords pair
        ;; 3. split the pair
        (while (not (= (length via-coords) resolution-num))
          (let ((distances (mapcar #'(lambda (prev next)
                                       (coords-distance prev next))
                                   (butlast via-coords) (cdr via-coords))))
            (let ((max-distance-index (max-index distances)))
              (let ((left-index (get (elt via-coords max-distance-index) :raw-index-value))
                    (right-index (get (elt via-coords (1+ max-distance-index)) :raw-index-value)))
                (let ((new-via-coords (send self :interpolate-raw (/ (+ left-index right-index) 2.0))))
                  ;; insert new-via-coords between left-index and right-index
                  (setq via-coords (append (subseq via-coords 0 (1+ max-distance-index))
                                           (list new-via-coords)
                                           (subseq via-coords (1+ max-distance-index) (length via-coords))))
                  )))))
        via-coords)))
  (:interpolate-raw (v)
    "Interpolate via-coords linearly.
Do not use this method directly. Use :interpolate method instead.
v should be 0 - (1- (length via-coords))"
    (if (or (< v 0) (> v (length via-coords)))
        (error "~A exceeds limit in interpolation" v))
    (let ((left-index (floor v))
          (right-index (ceiling v)))
      (if (= left-index right-index)
          ;; v should be integer
          (let ((c (send (elt via-coords left-index) :copy-worldcoords)))
            (setf (get c :raw-index-value) v)
            c)
        (let ((left-coords (send (elt via-coords left-index) :copy-worldcoords))
              (right-coords (send (elt via-coords right-index) :copy-worldcoords))
              (r (- v left-index)))
          (let ((c (midcoords r left-coords right-coords)))
            (setf (get c :raw-index-value) v)
            c)))))
  (:interpolate (v)
    "Interpolate via-coords linearly.
v should be 0 - 1"
    ;; v should be 0 ~ 1
    (let ((vv (* v (1- (length via-coords)))))
      (send self :interpolate-raw vv)))
  (:draw-on (&key (flush))
    (send-all min-via-coords :flush flush))
  (:draw (vwer)
    ;; support objects
    (gl::glPushAttrib gl::GL_ALL_ATTRIB_BITS)
    (gl::glDisable gl::GL_LIGHTING)
    (gl::glLineWidth (get self :line-width))
    (gl::glBegin gl::GL_LINE_STRIP)
    (let ((c (gl::find-color (get self :face-color))))
      (dotimes (i (length via-coords))
        (gl::glColor3fv c)
        (gl::glVertex3fv (send (elt via-coords i) :worldpos)))
      (gl::glEnd))
    (gl::glEnable gl::GL_LIGHTING)
    (gl::glPopAttrib))
  (:worldcoords ()
    (send self :interpolate 0.5))
  (:vertices ()
    (send-all via-coords :worldpos))
  (:to-marker (frame-id)
    (let ((marker (instance visualization_msgs::Marker :init)))
      (send marker :header :frame_id frame-id)
      (send marker :type visualization_msgs::Marker::*LINE_STRIP*)
      (send marker :scale :x 0.01)
      (send marker :points (mapcar #'(lambda (c)
                                       (ros::pos->tf-point (send c :worldpos)))
                                   via-coords))
      (send marker :color :r 1.0)
      (send marker :color :a 1.0)
      marker))
  )

(defclass ik-parameter
  :super propertied-object
  :slots ())

(defmethod ik-parameter
  (:init ()
    )
  )

(defun setup-move-targets (r)
  (let ((targets (list (cons :door-manipulation-move-target
                             ;;(make-coords :pos (float-vector 100 0 0)
                             (make-coords :pos (float-vector 50 0 0)
                                          :rpy (float-vector 0 0 pi))))))
    (dolist (target targets)
      (let ((mt (send (send r :rarm :end-coords :parent) :assoc
                      (make-cascoords
                       :coords (send (send (send r :rarm :end-coords) :copy-worldcoords)
                                     :transform (cdr target))))))
        (setf (get (send r :rarm :end-coords) (car target)) mt))))
  )

;; demo codes
(defun demo-basic-trajectory ()
  (let ((c (make-cube 100 100 100)))
    ;;(objects (list c))
    (setq *traj* (instance manipulation-trajectory :init
                           :via-coords (list (make-cascoords :pos (float-vector 0 0 0))
                                             (make-cascoords :pos (float-vector 100 0 0))
                                             (make-cascoords :pos (float-vector 200 0 0))
                                             (make-cascoords :pos (float-vector 200 100 0))
                                             (make-cascoords :pos (float-vector 200 100 100))
                                             (make-cascoords :pos (float-vector 300 100 100)))
                           :parent-object c))
    (dotimes (i 100)
      (let ((v (/ i 100.0)))
        (send (send *traj* :interpolate v) :draw-on :flush t)
        )
      )
  ))

(defun make-coords-relative (parent &rest args)
  (make-cascoords
   :coords (send (send parent :copy-worldcoords) :transform (apply #'make-coords args))))

(defun door-trajectory ()
  ;; require *door*
  (let* ((door-tmp (instance param-door :init 700 60 100 20
                             (float-vector 0 -640 950)
                             (float-vector -60 -600 950)
                             :use-leg t
                             :handle-l/r :right))
         (knob-coords (send door-tmp :handle :knob-handle)))
    (setq *traj* (instance manipulation-trajectory :init
                           :via-coords
                           (list (make-coords-relative knob-coords :pos (float-vector -100 200 -100) :rpy (float-vector 0 0 0))
                                 (make-coords-relative knob-coords :pos (float-vector 0 50 -50) :rpy (float-vector 0 0 0))
                                 (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0))
                                 (progn
                                   (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (progn
                                   (send (send door-tmp :joint :door-knob-joint) :joint-angle -70)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0))
                                   )
                                 (progn
                                   (send (send door-tmp :joint :door-hinge-joint) :joint-angle -10)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (progn
                                   (send (send door-tmp :joint :door-hinge-joint) :joint-angle -20)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (progn
                                   (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (progn
                                   (send (send door-tmp :joint :door-knob-joint) :joint-angle 0)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (make-coords-relative knob-coords :pos (float-vector 0 75 50) :rpy (float-vector 0 0 0))
                                 (make-coords-relative knob-coords :pos (float-vector 0 150 50):rpy (float-vector 0 0 0))
                                 (make-coords-relative knob-coords :pos (float-vector -100 150 50) :rpy (float-vector 0 0 0)))
                           :parent-object (send *door* :handle :knob-handle)))))

(defun door-trajectory-jaxon ()
  ;; require *door*
  (let* ((door-tmp (instance param-door :init 700 60 100 20
                             (float-vector 0 -640 950)
                             (float-vector -60 -600 950)
                             :use-leg t
                             :handle-l/r :right))
         (knob-coords (send door-tmp :handle :knob-handle)))
    (setq *traj* (instance manipulation-trajectory :init
                           :via-coords
                           (list (make-coords-relative knob-coords :pos (float-vector -200 -25 -50) :rpy (float-vector 0 0 0))
                                 ;;(make-coords-relative knob-coords :pos (float-vector 0 50 -50) :rpy (float-vector 0 0 0))
                                 (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0))
                                 (progn
                                   (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 ;; (progn
                                 ;;   (send (send door-tmp :joint :door-knob-joint) :joint-angle -70)
                                 ;;   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0))
                                 ;;   )
                                 (progn
                                   (send (send door-tmp :joint :door-hinge-joint) :joint-angle -10)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (progn
                                   (send (send door-tmp :joint :door-hinge-joint) :joint-angle -20)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (progn
                                   (send (send door-tmp :joint :door-knob-joint) :joint-angle -35)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 (progn
                                   (send (send door-tmp :joint :door-knob-joint) :joint-angle 0)
                                   (make-coords-relative knob-coords :pos (float-vector 0 -25 -50) :rpy (float-vector 0 0 0)))
                                 ;; (make-coords-relative knob-coords :pos (float-vector 0 75 50) :rpy (float-vector 0 0 0))
                                 ;; (make-coords-relative knob-coords :pos (float-vector 0 150 50):rpy (float-vector 0 0 0))
                                 ;; (make-coords-relative knob-coords :pos (float-vector -100 150 50) :rpy (float-vector 0 0 0))
                                 )
                           :parent-object (send *door* :handle :knob-handle)))))


(defun demo-manip-trajectory ()
  (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (load "package://drc_task_common/euslisp/test-drc-door-task.l")
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (unless (boundp '*robot*)
    (setq *robot* (instance hrp2jsknt-robot :init)))
  (setup-move-targets *robot*)
  ;; setup trajectory
  (setq *traj* (door-trajectory))
  (setf (get *traj* :face-color) :red)
  (send *door* :locate (float-vector 600 100 0) :world)
  (if (boundp '*irtviewer*)
      (send *irtviewer* :objects (list *robot* *door* *traj*))
    (objects (list *robot* *door* *traj*)))
  (send *robot* :reset-pose)
  (dolist (c (send *traj* :via-coords 100))
    (unless (send *robot* :inverse-kinematics c
                  :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                  :link-list (send *robot* :link-list (send (get (send *robot* :rarm :end-coords) :door-manipulation-move-target) :parent))
                  :debug-view :no-message)
      (return-from demo-manip-trajectory nil)))
  )

;; (load "package://drc_task_common/euslisp/trajectory.l")
