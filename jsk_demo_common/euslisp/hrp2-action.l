(require :detection-interface "package://jsk_perception/euslisp/detection_interface.l")
(load "package://pr2eus/speak.l")

(setq *detection-topic* "/camera_remote/rgb/ObjectDetection")
(setq *base-frame-id* "/BODY")

(if (and (boundp '*use-voicetext*) *use-voicetext*)
    (cond
     ((and (boundp '*use-english*) *use-english*)
      (setq *frige-speak-str* "the refrigerator"))
     (t (setq *frige-speak-str* "冷蔵庫")))
  (setq *frige-speak-str* "れいぞうこ"))

(setq *target-detect-coords-world* (make-coords :pos (float-vector 519.174 -2.5337 1111.02) :rpy (list 0.225849 -0.026944 0.008662)))
;; (setq *target-detect-coords-world* #<coordinates #Xce0a0e8  509.765 5.591 1115.908 / 0.306 0.033 0.025>)

(defun go-pos-for-open-fridge-door
  (&key (debug? t))
  (let* (detect-ret
         detect-coords-local
         detect-coords-world
         body-coords
         detect-coords-relative-to-target
         go-pos-x go-pos-y go-pos-theta
         )
    ;; detect fridge handle
    (setq detect-ret (check-detection :type "fridge" :speak-name *frige-speak-str*))
    (unless detect-ret
      (if (and (boundp '*use-voicetext*) *use-voicetext*)
          (cond
           ((and (boundp '*use-english*) *use-english*)
            (speak-jp "I could not find the refrigerator."))
           (t (speak-jp "冷蔵庫を見つけられませんでした。")))
        (speak-jp "れいぞうこ を みつけられません でした"))
      (return-from go-pos-for-open-fridge-door nil)
      )

    ;; transform detected coords
    (setq detect-coords-local (send (send detect-ret :worldcoords) :copy-worldcoords))
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq body-coords (send (send (car (send *robot* :links)) :worldcoords) :copy-worldcoords))
    (setq detect-coords-world (send body-coords :transform detect-coords-local :local))
    (when debug?
      (send detect-coords-world :draw-on :flush t :size 100 :width 5 :color #f(1 0 0)))

    ;; go-pos to fridge handle
    (setq detect-coords-relative-to-target
          (send (send (send detect-coords-world :copy-worldcoords) :inverse-transformation) :transformation
                (send (send *target-detect-coords-world* :copy-worldcoords) :inverse-transformation) :local))
    (setq go-pos-x (* 0.001 (elt (send detect-coords-relative-to-target :worldpos) 0)))
    (setq go-pos-y (* 0.001 (elt (send detect-coords-relative-to-target :worldpos) 1)))
    (setq go-pos-theta (elt (mapcar #'rad2deg (car (rpy-angle (send detect-coords-relative-to-target :worldrot)))) 0))
    (when debug?
      (warning-message 2 "go-pos to fridge  x: ~a  y:~a  theta: ~a~%"
                       go-pos-x go-pos-y go-pos-theta))
    (when (y-or-n-p "send go-pos to real robot?")
      (send *ri* :go-pos go-pos-x go-pos-y go-pos-theta))
    t))


(defun open-fridge-door
  (&key (debug? t))

  ;; reset-manip-pose
  (send *robot* :reset-manip-pose)
  (send *robot* :head :neck-p :joint-angle 0)
  (model2real)

  ;; go pos to fride
  (dotimes (i 2)
    (unless (go-pos-for-open-fridge-door)
      (return-from open-fridge-door nil))
    )

  ;; pre-reach with rarm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -57.9275 -17.6226 44.8409 -66.6477 -43.5954 5.08905 15.4178 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
  (model2real)

  ;; reach with rarm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -65.1089 5.64428 52.2818 -48.2923 -38.8337 -5.17672 11.2115 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
  (model2real)

  ;; open1 with rarm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -51.777 -19.7412 48.0005 -86.6959 -50.8279 13.0751 22.5508 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
  (model2real)

  ;; open2 with rarm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -48.4909 -25.545 46.3891 -94.3029 -54.737 16.5416 24.1182 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
  (model2real)

  ;; release1 with rarm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -47.0618 -39.2662 39.5824 -89.2953 -62.8388 32.5249 20.8336 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
  (model2real)

  ;; release2 with rarm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 40.0 -49.1977 -47.4318 36.1815 -74.5764 -67.3357 51.7533 17.5418 60.0 50.0 30.0 10.0 -120.0 25.0 5.0 -20.0 -60.0))
  (model2real)

  ;; reset-manip-pose
  (send *robot* :reset-manip-pose)
  (model2real)

  ;; reach with larm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 -40.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 10.4676 37.943 -7.23985 -100.712 31.5252 8.94768 -6.74242 -60.0))
  (model2real)

  ;; open with larm
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 -40.0 0.0 0.0 40.0 50.0 -30.0 -10.0 -120.0 -25.0 -5.0 -20.0 60.0 -35.5539 0.289376 -50.0762 -62.5947 22.2163 6.94033 -0.540074 -60.0))
  (model2real :tm 1000)

  ;; reset-manip-pose
  (send *robot* :reset-manip-pose)
  (model2real)

  t)

(defun move-from-frontfridge-to-pregrasp
  (&key (debug? t))
  ;; go-pos to look can place 1
  (send *ri* :go-pos 0 -0.1 10)

  ;; look pose
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 30.748 -28.2643 5.65972 -130.45 -14.3662 -17.8409 4.76971 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  ;; go-pos to look can place 2
  (send *ri* :go-pos 0.2 0 0)
  t)

(defun grasp-can
  (&key (debug? t))

  (let* (detect-ret
         detect-coords-local
         detect-coords-world
         body-coords
         reach-pos
         pos-diff-between-current-and-target
         )
    ;; detect georgia
    (setq detect-ret (check-detection :type "georgia" :speak-name "georgia"))
    (unless detect-ret
      (if (and (boundp '*use-voicetext*) *use-voicetext*)
          (cond
           ((and (boundp '*use-english*) *use-english*)
            (speak-jp "I could not find the georgia."))
           (t (speak-jp "ジョージアを見つけられませんでした。")))
        (speak-jp "ジョージア を みつけられません でした"))
      (return-from grasp-can nil)
      )

    ;; transform detected coords
    (setq detect-coords-local (send (send detect-ret :worldcoords) :copy-worldcoords))
    (send *robot* :angle-vector (send *ri* :state :potentio-vector))
    (send *robot* :fix-leg-to-coords (make-coords))
    (setq body-coords (send (send (car (send *robot* :links)) :worldcoords) :copy-worldcoords))
    (setq detect-coords-world (send body-coords :transform detect-coords-local :local))
    (when debug?
      (send detect-coords-world :draw-on :flush t :size 100 :width 5 :color #f(1 0 0)))

    (send *robot* :reset-pose)
    (send *robot* :fix-leg-to-coords (make-coords))

    ;; pre-grasp
    (send *robot* :hand :rarm :hook-pose)
    (hand-model2real)

    ;; reach into fridge
    (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 -45.3312 -23.9005 14.9548 -100.353 -21.5289 -11.7091 47.7505 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
    (model2real)

    ;; pre-reach to can
    (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 30.0 0.0 -20.0 10.0 -27.9637 -43.6456 -13.4299 -98.5874 -17.6826 -19.3391 44.3072 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
    (model2real)

    ;; reach to can
    ;; (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 30.0 0.0 -20.0 10.0 -48.6776 -35.9706 6.61446 -79.2443 -18.6859 -16.0366 35.9212 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
    (setq reach-pos (send detect-coords-world :worldpos))
    (setf (elt reach-pos 2) 1200)
    (setq pos-diff-between-current-and-target
          (v- reach-pos (send (send *robot* :rarm :end-coords) :worldpos)))
    (send *robot* :rarm :move-end-pos pos-diff-between-current-and-target :world :rotation-axis :z)
    (model2real)

    ;; grasp
    (send *robot* :hand :rarm :angle-vector #f(90.0 0.0 0.0 10.0 -20.0 90.0))
    (hand-model2real)

    ;; move back
    (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 -45.3312 -23.9005 14.9548 -100.353 -21.5289 -11.7091 47.7505 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
    (model2real)

    ;; go-pos pose
    (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 10.0 30.748 -28.2643 5.65972 -130.45 -14.3662 -17.8409 4.76971 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
    (model2real)

    t))

(defun move-from-pregrasp-to-preclose
  (&key (debug? t))
  ;; go-pos 1
  (send *ri* :go-pos 0 0 10)

  ;; go-pos 2
  (send *ri* :go-pos -0.2 -0.8 0)

  ;; go-pos 3
  (send *ri* :go-pos -0.2 -0.2 45)

  t)

(defun close-fridge
  (&key (debug? t))

  ;; pre-pose 1
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 11.5172 -27.0874 -26.5744 -98.4134 -21.4793 -36.5186 6.85352 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  ;; pre-pose 2
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 -50.0714 -26.5624 -3.23559 -26.9182 -13.6261 -14.5182 -12.8529 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  ;; go-pos 4
  (send *ri* :go-pos 0.1 0 0)

  ;; pre-pose 3
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 -50.0714 -26.5624 -3.23559 -26.9182 -13.6261 50.0 -12.8529 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  ;; close
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 40.0 0.0 -20.0 10.0 -43.1154 -18.0471 -6.98633 -44.7013 -16.7301 63.0793 -0.662186 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real :tm 2000)

  ;; go-pos pose
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 0.0 10.0 30.748 -28.2643 5.65972 -130.45 -14.3662 -17.8409 4.76971 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  t)

(defun approach-to-target
  (obj &key (debug? t))
  (let* (go-pos-x go-pos-y go-pos-theta)
    (setq go-pos-x (- (* 0.001 (elt (send obj :worldpos) 0)) 1.0))
    (setq go-pos-y (* 0.001 (elt (send obj :worldpos) 1)))
    (setq go-pos-theta (rad2deg (atan2 go-pos-y go-pos-x)))
    (when debug?
      (warning-message 2 "go-pos to fridge  x: ~a  y:~a  theta: ~a~%"
                       go-pos-x go-pos-y go-pos-theta))
    (when (y-or-n-p "send go-pos to real robot?")
      (send *ri* :go-pos go-pos-x go-pos-y go-pos-theta))
    t))

(defun place-can
  (&key (debug? t))
  ;; pre-reach to table
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 13.6057 -31.2966 19.122 -99.5359 -22.2677 -12.1836 -14.0861 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  ;; reach to table
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 20.0 0.0 -20.0 10.0 -26.8427 -4.93447 19.3492 -27.7585 -20.0073 -12.7064 -38.9075 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  ;; release
  (send *robot* :hand :rarm :hook-pose)
  (hand-model2real)

  ;; pre-reach to table
  (send *robot* :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -26.0 50.0 -24.0 0.0 0.0 0.0 0.0 -20.0 10.0 13.6057 -31.2966 19.122 -99.5359 -22.2677 -12.1836 -14.0861 60.0 10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0))
  (model2real)

  (send *robot* :reset-manip-pose)
  (send *robot* :head :neck-p :joint-angle 0)
  (model2real)
  t)


;; utils ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun model2real
  (&key (tm 5000) (wait? t))
  "send angle-vector.
   arguments:
     tm: time of interpolation [msec]
  "
  (send *ri* :angle-vector (send *robot* :angle-vector) tm)
  (when wait? (send *ri* :wait-interpolation))
  )

(defun hand-model2real
  (&key (wait? t))
  "send hand angle-vector.
   arguments:
     tm: time of interpolation [msec]
  "
  (send *ri* :hand-angle-vector (apply #'concatenate float-vector (send-all (send *robot* :hand :arms) :angle-vector)))
  (when wait? (send *ri* :hand-wait-interpolation))
  )
