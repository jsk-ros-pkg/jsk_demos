;; TF utilities for pot/handle detections.
;; This file converts detection centers from the detection frame to the
;; EusLisp model world frame. Input and output detection centers are in meters.
;; Detection format is:
;; (class-name center dimensions volume box-msg label-msg label-index)

(setq *pot-handle-tf-listener* nil)
(setq *pot-handle-target-frame* "/base_link")
(setq *pot-handle-tf-timeout* 1)

(defun init-pot-handle-tf-listener ()
  (unless *pot-handle-tf-listener*
    (setq *pot-handle-tf-listener*
          (instance ros::transform-listener :init)))
  *pot-handle-tf-listener*)

(defun pot-handle-detection-frame-id (detection)
  (let ((box (pot-handle-detection-box detection)))
    (when box
      (send box :header :frame_id))))

(defun pot-handle-detection-stamp (detection)
  (let ((box (pot-handle-detection-box detection)))
    (when box
      (send box :header :stamp))))

(defun pot-handle-vector-m->mm (vec)
  (float-vector (* 1000.0 (elt vec 0))
                (* 1000.0 (elt vec 1))
                (* 1000.0 (elt vec 2))))

(defun pot-handle-vector-mm->m (vec)
  (float-vector (/ (elt vec 0) 1000.0)
                (/ (elt vec 1) 1000.0)
                (/ (elt vec 2) 1000.0)))

(defun pot-handle-transform-vector-to-base (vec-m source-frame stamp)
  (let ((listener (init-pot-handle-tf-listener)))
    (when (send listener
                :wait-for-transform
                *pot-handle-target-frame*
                source-frame
                stamp
                *pot-handle-tf-timeout*)
      (let* ((source-to-base
              (send listener
                    :lookup-transform
                    *pot-handle-target-frame*
                    source-frame
                    stamp))
             (vec-mm (pot-handle-vector-m->mm vec-m)))
        (pot-handle-vector-mm->m
         (send source-to-base :transform-vector vec-mm))))))

(defun pot-handle-transform-vector-base-to-model (vec-m)
  (let ((vec-mm (pot-handle-vector-m->mm vec-m)))
    (pot-handle-vector-mm->m
     (send (send *pr2* :worldcoords) :transform-vector vec-mm))))

(defun pot-handle-transform-vector-to-model (vec-m source-frame stamp)
  (let ((base-vec-m
         (pot-handle-transform-vector-to-base vec-m source-frame stamp)))
    (when base-vec-m
      (pot-handle-transform-vector-base-to-model base-vec-m))))

(defun make-transformed-pot-handle-detection (detection center-m)
  (list (pot-handle-detection-class detection)
        center-m
        (pot-handle-detection-dimensions detection)
        (pot-handle-detection-volume detection)
        (pot-handle-detection-box detection)
        (elt detection 5)
        (elt detection 6)))

(defun transform-pot-handle-detection-to-model (detection)
  (let* ((source-frame (pot-handle-detection-frame-id detection))
         (stamp (pot-handle-detection-stamp detection))
         (center (pot-handle-detection-center detection))
         (model-center nil))
    (when (and source-frame stamp center)
      (setq model-center
            (pot-handle-transform-vector-to-model
             center
             source-frame
             stamp)))
    (when model-center
      (make-transformed-pot-handle-detection detection model-center))))

(defun transform-pot-handle-detections-to-model (detections)
  (let ((transformed nil))
    (dolist (detection detections)
      (let ((model-detection
             (transform-pot-handle-detection-to-model detection)))
        (when model-detection
          (setq transformed (cons model-detection transformed)))))
    (reverse transformed)))

(defun transform-pot-handle-selection-to-model (selection)
  (let ((target-pot (selected-target-pot selection))
        (left-handle (selected-left-handle selection))
        (right-handle (selected-right-handle selection)))
    (when (and target-pot left-handle right-handle)
      (let ((model-pot (transform-pot-handle-detection-to-model target-pot))
            (model-left (transform-pot-handle-detection-to-model left-handle))
            (model-right (transform-pot-handle-detection-to-model right-handle)))
        (when (and model-pot model-left model-right)
          (list model-pot model-left model-right))))))
