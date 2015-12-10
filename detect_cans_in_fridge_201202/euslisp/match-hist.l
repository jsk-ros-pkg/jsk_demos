;;(jsk)
;; 色ヒストグラムのマッチ関数

;; モデル： n分割した表面それぞれの色相彩度のヒストグラム
;; 入力： 見えている点群全体の色相彩度のヒストグラム

;; for rgb2hsi
;;(setq ros::*compile-message* nil)
;;(load "package://euslib/jsk/jskgeo.l")

;; copy from color_histogram_creater.l
(defun get-bin (d &key (step 3) (max-num 1.0) (min-num 0.0))
  (if (>= d max-num) (return-from get-bin (- step 1)))
  (let ((ret (floor (/ d (/ (- max-num min-num) step)))))
    (if (>= ret step) (return-from get-bin (- step 1)))
    (if (<= ret 0) (return-from get-bin 0))
    ret))

#| ;; move to irtpointcloud.l
(defmethod pointcloud
  (:generate-color-histogram-hs (&key (h-step 3) (s-step 3))
    (let ((col (send self :colors))
          (hist (make-array
                 (list h-step s-step)
                 :element-type float-vector)))
      (dotimes (i (send self :size))
        (let* ((his (rgb2his (matrix-row col i)))
               (h-bin (get-bin (elt his 0) :step h-step :max-num 360.0))
               (s-bin (get-bin (elt his 2) :step s-step)))
          (incf (aref hist h-bin s-bin))))
      (scale-matrix (/ 1.0 (send self :size)) hist))))
|#
(defun make-histdata(pclst)
  (flatten
   (mapcar
    #'(lambda(pc)
        (coerce (array-entity (send pc :generate-color-histogram-hs
                                    :h-step 10 :s-step 10)) cons))
    pclst)))

;; utils
(defun maxidx (vec)
  (let ((idx 0))
    (dotimes (i (length vec) idx)
      (when (< (elt vec idx) (elt vec i))
        (setq idx i)))))

(defun rotate-h-bin (hist &key (cw t) (h-step 9) (s-step 7))
  (if (matrixp hist) (setq hist (array-entity hist)))
  (let ((hret (copy-seq hist)))
    (cond
     (cw
      (dotimes (hi (1- h-step))
        (dotimes (si s-step)
          (setf (elt hret (+ (* (1+ hi) s-step) si))
                (elt hist (+ (* hi      s-step) si)))
          ))
      (dotimes (si s-step)
        (setf (elt hret                           si)
              (elt hist (+ (* (1- h-step) s-step) si))))
      )
     (t ;; ccw
      (dotimes (hi (1- h-step))
        (dotimes (si s-step)
          (setf (elt hret (+ (* hi      s-step) si))
                (elt hist (+ (* (1+ hi) s-step) si)))
          ))
      (dotimes (si s-step)
        (setf (elt hret (+ (* (1- h-step) s-step) si))
              (elt hist (+                        si))))
      ))
    hret
    ))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; 単純に内積を取る
(defun color-hist-similarity1 (input models &optional)
  (apply #'max (mapcar #'(lambda(m) (v. (coerce input float-vector) (coerce m float-vector))) models))
  )
;; Swain,1991で提案されているHistogram Intersection
(defun color-hist-similarity-hi (input models)
  (apply #'max (mapcar #'(lambda(m)(apply #'+ (mapcar #'min input m))) models))
  )
;; kl-div
(defun kl-divergence (f g)
;;  (apply #'+ (mapcar #'(lambda(x y) (* x (/ (log (+ x 1e-9)) (log (+ y 1e-9))))) f g)))
  (apply #'+ (mapcar #'(lambda(x y) (* x (log (/ (+ x 1e-9) (+ y 1e-9))))) f g)))
(defun color-hist-similarity-kl (input models)
  (exp (- (/
           (apply #'min (mapcar #'(lambda(m) (kl-divergence m input)) models)) ;; ??
           3.0))))

;; euclidean distance
(defun color-hist-distance (input models &optional (thr 1))
  (apply #'min (mapcar #'(lambda(m)(distance input m)) models))
  )
;; 代表色と２次的特徴による照合アルゴリズム
(defun color-hist-distance2 (input models)
  (dolist (mod models)
    (when (eq (maxidx input) (maxidx mod))
      ()))
  )

(defun match-model (objs model-name &key (thr 0.67) (rotate 20))
  (let ((histnum (ros::get-param "/object_hs_histgram/histnum"))
        (hs_bin (ros::get-param "/object_hs_histgram/hs_bin"))
        (model-hist (ros::get-param model-name))
        model-hist-list ret hist hist-cw hist-ccw
        (val 0))
    (format t "TARGET: ~A~%" model-name)
    (dotimes (i histnum)
      (push (subseq model-hist (* i (apply #'* hs_bin)) (* (1+ i) (apply #'* hs_bin)))
            model-hist-list))
    ;;
    (dolist (obj objs)
      (setq hist (send obj :generate-color-histogram-hs
                       :h-step (elt hs_bin 0) :s-step (elt hs_bin 1)
                       :sizelimits (* 0.2 (send obj :size))))
      (when hist
        (setq hist-cw (send obj :generate-color-histogram-hs
                            :h-step (elt hs_bin 0) :s-step (elt hs_bin 1)
                            :sizelimits (* 0.2 (send obj :size))
                            :rotate-hue rotate))
        (setq hist-ccw (send obj :generate-color-histogram-hs
                             :h-step (elt hs_bin 0) :s-step (elt hs_bin 1)
                             :sizelimits (* 0.2 (send obj :size))
                             :rotate-hue (- rotate)))
        (setq hist     (coerce (array-entity hist) cons))
        (setq hist-cw  (coerce (array-entity hist-cw) cons))
        (setq hist-ccw (coerce (array-entity hist-ccw) cons))
        (let ((ev-cw  (color-hist-similarity-kl hist-cw model-hist-list))
              (ev-ccw (color-hist-similarity-kl hist-ccw model-hist-list))
              (ev-org (color-hist-similarity-kl hist model-hist-list))
              ev)
          (setq ev (max ev-cw ev-org ev-ccw))
          ;; choose max
          (format t "~A -> ~A (~A/~A/~A)~%" obj ev ev-ccw ev-org ev-cw)
          (when (and (< thr ev) (< val ev))
            (setq val ev ret obj)))
        ))
    (format t "RESULT(~A): ~A -> ~A~%" model-name ret val)
    (if ret
        (setf (get ret :results)
              (append (get ret :results) (list (cons model-name val)))))
    ret
    ))

(defun visualize-histogram (hist &key
                                 ((:h-step hbin) 9) ((:s-step sbin) 7)
                                 (box-size 40)
                                 (hlimits (cons 360.0 0.0))
                                 (vlimits (cons 1.0 0.15))
                                 (slimits (cons 1.0 0.25))
                                 (sizelimits 1))
  (let (ret
        (hist-vec (if (matrixp hist) (array-entity hist) hist)))
    (dotimes (hi hbin)
      (let ((ch (+ (* (/ (- (car hlimits) (cdr hlimits)) hbin) (+ hi 0.5))
                   (cdr hlimits))))
        (dotimes (si sbin)
          (let* ((hcol (list ch 1.0
                             (+ (* (/ (- (car slimits) (cdr slimits)) sbin) (+ si 0.5))
                                (cdr slimits))))
                 (col (scale (/ 1.0 255) (coerce (hvs2rgb hcol) float-vector)))
                 (cube-height (* (* hbin sbin box-size)
                                 (elt hist-vec (+ (* hi sbin) si))))
                 (cu (make-cube box-size box-size cube-height))
                 )
            (when (/= cube-height 0.0)
              (send cu :translate
                    (float-vector (* box-size hi) (* box-size si) (/ cube-height 2)))
              (send cu :set-color col)
              (push cu ret))
            ))))
  ret))

#|
(defun get-peaks (hist s1 s2)
  (let (ret var)
  (dotimes (y s2)
    (dotimes (x s1)
      (setq var (elt hist (+ x (* y s1))) tmp nil)
      (dolist (dx '(-1 0 1))
	(dolist (dy '(-1 0 1))
	  (push (elt hist (+ (mod (+ x dx s1) s1) (* (mod (+ y dy s2) s2) s1))) tmp)))
      (when (< 0 (apply #'max tmp) (+ var 1e-5))
	(push (list (apply #'+ tmp) x y) ret))))
  (sort ret #'(lambda(x y)(> (car x) (car y))))
  ))
(defun visualize-obj-hist (obj)
  (let ((hbin 10) (sbin 10) hist peaks cols cubes)
    (setq hist (coerce (array-entity
                        (send obj :generate-color-histogram-hs
                              :h-step hbin :s-step sbin)) cons))
    (setq peaks (get-peaks hist hbin sbin))
    (setq cols (mapcar #'(lambda(p)(hvs2rgb
                                    (* (/ 2pi hbin) (+ (elt p 1) 0.5))
                                    1.0
                                    (* (/ 2pi sbin) (+ (elt p 2) 0.5))))
                       peaks))
    (dolist (col cols)
      (let ((cu (make-cube 100 100 100)))
        (send-all cu :translate #f(0 0 100))
        (push cu cubes)
        (send cu :translate (car (send obj :point-list)))
        (send cu :set-color col)
        ))
    cubes))
|#

#|
(defvar *model-names* (ros::get-param "/object_hs_histgram/names"))
(dolist (nm *model-names*)
  (setq obj (match-model obj-lst (format nil "/object_hs_histgram/~A" nm)))
  (when obj
    (objects (list obj))
    (read-line))
  )
|#
;(hoge obj-lst "/object_hs_histgram/mountain_dew")
;;"/object_hs_histgram/georgia_emerald_coffee"
;;"/object_hs_histgram/mets"
