;;Okada-sensei seminar practice task ;;last editted oct 17 ;; Michitoshi TSUBAKI
;;building specific motions using euslisp command

(require "package://pr2eus/pr2.l")  ;;import pr2 package
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l") 
(if (not (boundp '*pr2*)) (pr2-init)) ;; (setq *pr2* (pr2)))  ;; (pr2) is alternated by  *pr2*
;;boudp 'xx : xxという変数に値が代入されているか
;;setq xx yy : xxとう変数にyyという値を代入する

(send *pr2* :reset-pose) ;; 初期姿勢
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :stop-grasp :arms)
(send *ri* :wait-interpolation)


(objects (list *pr2*))

;;徐々に腕（肩）を広げる(Oct 17)
(send *pr2* :larm :shoulder-p :joint-angle 74)
(send *pr2* :larm :shoulder-p :joint-angle 74)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)

(send *pr2* :larm :shoulder-p :joint-angle 54)
(send *pr2* :rarm :shoulder-p :joint-angle 54)

(send *pr2* :larm :gripper :joint-angle 30)
(send *pr2* :rarm :gripper :joint-angle 30)

(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :start-grasp :arms)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 34)
(send *pr2* :rarm :shoulder-p :joint-angle 34)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 20)
(send *pr2* :rarm :shoulder-p :joint-angle 20)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle 0)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


;;徐々に型を下げる(Oct 22)
(send *pr2* :rarm :shoulder-r :joint-angle -80)
(send *pr2* :larm :shoulder-r :joint-angle 80)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :rarm :shoulder-r :joint-angle -95)
(send *pr2* :larm :shoulder-r :joint-angle 95)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :rarm :shoulder-r :joint-angle -110)
(send *pr2* :larm :shoulder-r :joint-angle 100)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)



(send *pr2* :rarm :shoulder-r :joint-angle -130)
(send *pr2* :larm :shoulder-r :joint-angle 120)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)




(send *pr2* :larm :elbow-p :joint-angle -110)
(send *pr2* :rarm :elbow-p :joint-angle -100)
(send *pr2* :rarm :shoulder-r :joint-angle -130)
(send *pr2* :larm :shoulder-r :joint-angle 120)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *irtviewer* :draw-objects)
(unix:sleep 1)


(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :rarm :shoulder-r :joint-angle -130)
(send *pr2* :larm :shoulder-r :joint-angle 120)
(send *pr2* :larm :elbow-p :joint-angle -100)
(send *pr2* :rarm :elbow-p :joint-angle -70)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 30)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)




(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :rarm :shoulder-r :joint-angle -100)
(send *pr2* :larm :shoulder-r :joint-angle 100)
(send *pr2* :larm :elbow-p :joint-angle -100)
(send *pr2* :rarm :elbow-p :joint-angle -70)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 30)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :rarm :shoulder-r :joint-angle -80)
(send *pr2* :larm :shoulder-r :joint-angle 80)
(send *pr2* :larm :elbow-p :joint-angle -110)
(send *pr2* :rarm :elbow-p :joint-angle -100)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 30)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :rarm :shoulder-r :joint-angle -100)
(send *pr2* :larm :shoulder-r :joint-angle 100)
(send *pr2* :larm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 30)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


;;fixing now last edited Oct 22 by Michitoshi TSUBAKI

(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :rarm :shoulder-r :joint-angle -100)
(send *pr2* :larm :shoulder-r :joint-angle 100)
(send *pr2* :larm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 0)
(send *pr2* :rarm :wrist-r :joint-angle 0)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :rarm :shoulder-r :joint-angle -120)
(send *pr2* :larm :shoulder-r :joint-angle 120)
(send *pr2* :larm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 90)
(send *pr2* :rarm :wrist-r :joint-angle 90)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :torso :waist-z :joint-angle 200)
(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)


(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :rarm :shoulder-p :joint-angle -20)
(send *pr2* :rarm :shoulder-r :joint-angle -120)
(send *pr2* :larm :shoulder-r :joint-angle 120)
(send *pr2* :larm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-p :joint-angle -120)
(send *pr2* :rarm :elbow-r :joint-angle 0)
(send *pr2* :rarm :wrist-p :joint-angle -40)
(send *pr2* :larm :wrist-r :joint-angle 0)
(send *pr2* :rarm :wrist-r :joint-angle 0)
(send *pr2* :larm :shoulder-p :joint-angle 10)
(send *pr2* :torso :waist-z :joint-angle 300)

(send *irtviewer* :draw-objects)
;;(unix:sleep 1)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
(send *ri* :wait-interpolation)



