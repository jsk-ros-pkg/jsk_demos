;; 色ヒストグラムのマッチ関数

;; モデル： n分割した表面それぞれの色相彩度のヒストグラム
;; 入力： 見えている点群全体の色相彩度のヒストグラム

;; for rgb2hsi
(load "package://euslib/jsk/jskgeo.l")

;; copy from color_histogram_creater.l
(defun get-bin (d &key (step 3) (max-num 1.0) (min-num 0.0))
 (if (= d max-num) (- step 1)
   (floor (/ d (/ (- max-num min-num) step)))))

(defmethod pointcloud
  (:hs-hist (&key (h-step 3) (s-step 3))
    (let ((col (send self :colors))
          (hist (make-array
                 (list h-step s-step)
                 :element-type float-vector)))
      (dotimes (i (send self :size))
        (let* ((his (geo:rgb2his (matrix-row col i)))
               (h-bin (get-bin (elt his 0) :step h-step :max-num 360.0))
               (s-bin (get-bin (elt his 2) :step s-step)))
          (incf (aref hist h-bin s-bin))))
      (scale-matrix (/ 1.0 (send self :size)) hist))))


;; utils
(defun maxidx (vec)
  (let ((idx 0))
    (dotimes (i (length vec) idx)
      (when (< (elt vec idx) (elt vec i))
	(setq idx i)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; 単純に内積を取る
(defun color-hist-similarity1 (input models &optional)
  (apply #'max (mapcar #'(lambda(m)(v. input m)) models))
  )
;; Swain,1991で提案されているHistogram Intersection
(defun color-hist-similarity-hi (input models)
  (apply #'max (mapcar #'(lambda(m)(apply #'+ (mapcar #'min input m))) models))
  )
;; kl-div
(defun kl-divergence (f g)
  (apply #'+ (mapcar #'(lambda(x y)(* x (/ (log (+ x 1e-9)) (log (+ y 1e-9))))) f g)))
(defun color-hist-similarity-kl (input models)
  (apply #'max (mapcar #'(lambda(m)(kl-divergence m input)) models)) ;; ??
  )

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

(defun match-model (objs model &optional (thr 0.7))
  (setq num (ros::get-param "/object_hs_histgram/histnum")
        bin (ros::get-param "/object_hs_histgram/hs_bin"))
  (format t "TARGET: ~A~%" model)
  (setq model (ros::get-param model) hist-mod nil)
  (dotimes (i num)
    (push (subseq model (* i (apply #'* bin)) (* (1+ i) (apply #'* bin))) hist-mod))
  ;;
  (setq ret nil hist nil val 0)
  (dolist (obj objs)
    (setq hist (coerce (array-entity (send obj :hs-hist :h-step (elt bin 0) :s-step (elt bin 1))) cons))
;    (let ((ev (color-hist-similarity-hi hist hist-mod)))
    (let ((ev (color-hist-similarity-kl hist hist-mod)))
      (format t "~A -> ~A~%" obj ev)
      (when (and (< thr ev) (< val ev))
	(setq val ev ret obj)))
    )
  (format t "RESULT: ~A -> ~A~%" ret val)
    ret
  )

(defun visualize-obj-hist (obj)
  (let ((hbin 10) (sbin 5) hist peaks cols cubes)
    (setq hist (coerce (array-entity (send obj :hs-hist :h-step hbin :s-step sbin)) cons))
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
	(send cu :set-color col)))
    cubes))

(defvar *model-names* (ros::get-param "/object_hs_histgram/names"))

#|
(dolist (nm model-names)
  (setq obj (match-model obj-lst (format nil "/object_hs_histgram/~A" nm)))
  (when obj
    (objects (list obj))
    (read-line))
  )
|#
;(hoge obj-lst "/object_hs_histgram/mountain_dew")
;;"/object_hs_histgram/georgia_emerald_coffee"
;;"/object_hs_histgram/mets"
