(require "euslib/irteus_proposals/motion-lib-proposal.l")

(defun reach
  (obj
   via-coords-offsets
   &key
   (real nil)
   (base-time 1000)
   (robot *robot*)
   (move-target (list (send robot :rarm :end-coords) (send robot :larm :end-coords)))
   (target-coords (list (send obj :handle :name :rarm) (send obj :handle :name :larm)))
   (base-time-list (make-list (1+ (length via-coords-offsets)) :initial-element (* base-time 2)))
   )
  ;; atom -> list
  (if (and via-coords-offsets (atom via-coords-offsets)) (setq via-coords-offsets (list via-coords-offsets)))
  (if (atom move-target) (setq move-target (list move-target)))
  (if (atom target-coords) (setq target-coords (list target-coords)))
  ;; argument check
  (unless (= (1+ (length via-coords-offsets)) (length base-time-list))
    (error ";; ERROR: list length differ : coords-list ~A base-time-list ~A~%"
          (1+ (length via-coords-offsets)) (length base-time-list))
    (return-from reach t))
  (unless (= (length move-target) (length target-coords))
    (error ";; ERROR: list length differ : move-target ~A target-coords ~A~%"
          (length move-target) (length target-coords))
    (return-from reach t))
  ;; solve inverse kinematics
  (let (avs)
    (setq via-coords-offsets (append via-coords-offsets (list (make-list (length move-target) :initial-element (make-coords)))))
    (dolist (offset via-coords-offsets)
      (if (atom offset) (setq offset (list offset)))
      (unless
          (fullbody-ik
           (mapcar #'(lambda (tc of)
                       (send (send tc :copy-worldcoords) :transform of))
                   target-coords offset)
           :robot robot
           :move-target move-target
           )
        (warn ";; reach ~A ik failed!!~%" (send obj :name))
        (return-from reach nil))
      (push (list (send robot :angle-vector) (send robot :hand-angle-vector)) avs)
      ;; send to real robot
      (when real
        (send *ri* :angle-vector (send robot :angle-vector) (elt base-time-list 0))
        (send *ri* :hand-angle-vector (send robot :hand-angle-vector))
        (send *ri* :wait-interpolation)
        (send *ri* :hand-wait-interpolation)))
    (mapcar #'(lambda (x y) (append x (list y))) (reverse avs) base-time-list)
    )
  )


(defun setup-test-reach
  ()
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
  (load "models/cardboard-box-object.l")
  (setq *robot* (instance hrp2jsknt-robot :init))
  (setq *obj* (make-cube 200 200 200))
  (send *obj* :put :rarm-handle
        (make-cascoords :coords (make-coords :pos (float-vector 0 -100 0))
                        :name :rarm-handle
                        :parent *obj*))
  (send *obj* :put :larm-handle
        (make-cascoords :coords (make-coords :pos (float-vector 0 100 0))
                        :name :larm-handle
                        :parent *obj*))
  (send *obj* :locate (float-vector 400 0 1000) :world)
  (objects (list *robot* *obj*))
  )

(defun test-reach-single-arm-no-via-coords
  ()
  (reach *obj* nil
         :real nil
         :move-target (send *robot* :larm :end-coords)
         :target-coords (send (send *obj* :get :larm-handle) :worldcoords)
         )
  )

(defun test-reach-single-arm-single-via-coords
  ()
  (reach *obj*
         (list (make-coords :pos (float-vector -50 50 0)))
         :real nil
         :move-target (send *robot* :larm :end-coords)
         :target-coords (send (send *obj* :get :larm-handle) :worldcoords)
         )
  )

(defun test-reach-single-arm-multi-via-coords
  ()
  (reach *obj*
         (list (make-coords :pos (float-vector -100 100 0))
               (make-coords :pos (float-vector -50 50 0)))
         :real nil
         :move-target (send *robot* :larm :end-coords)
         :target-coords (send (send *obj* :get :larm-handle) :worldcoords)
         )
  )

(defun test-reach-dual-arm-no-via-coords
  ()
  (reach *obj* nil
         :real nil
         :move-target (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
         :target-coords (list (send (send *obj* :get :rarm-handle) :worldcoords) (send (send *obj* :get :larm-handle) :worldcoords))
         )
  )

(defun test-reach-dual-arm-single-via-coords
  ()
  (reach *obj*
         (list (list (make-coords :pos (float-vector -50 -50 0)) (make-coords :pos (float-vector -50 50 0))))
         :real nil
         :move-target (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
         :target-coords (list (send (send *obj* :get :rarm-handle) :worldcoords) (send (send *obj* :get :larm-handle) :worldcoords))
         )
  )

(defun test-reach-dual-arm-multi-via-coords
  ()
  (reach *obj*
         (list (list (make-coords :pos (float-vector -100 -100 0)) (make-coords :pos (float-vector -100 100 0)))
               (list (make-coords :pos (float-vector -50 -50 0)) (make-coords :pos (float-vector -50 50 0))))
         :real nil
         :move-target (list (send *robot* :rarm :end-coords) (send *robot* :larm :end-coords))
         :target-coords (list (send (send *obj* :get :rarm-handle) :worldcoords) (send (send *obj* :get :larm-handle) :worldcoords))
         )
  )

(defun all-reach-test
  ()
  (setup-test-reach)
  (test-reach-single-arm-no-via-coords)
  (test-reach-single-arm-single-via-coords)
  (test-reach-single-arm-multi-via-coords)
  (test-reach-dual-arm-no-via-coords)
  (test-reach-dual-arm-single-via-coords)
  (test-reach-dual-arm-multi-via-coords)
  )

(warn ";; (all-reach-test)~%")
