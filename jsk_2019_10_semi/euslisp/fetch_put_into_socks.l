; (objects (list *fetch*))

; (send *fetch* :reset-pose)
; (send *irtviewer* :draw-objects)
; (format t "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa")
;
; ;; まず、手を前に出して、ものをつかませる
; (send *fetch* :angle-vector #f(45.2426 57.9818 3.49487 77.8199 -59.6255 10.2316 -78.2444 74.1795 0.27191 15.2598))
; (send *irtviewer* :draw-objects)
; (send *fetch* :go-grasp :pos 1)
; (send *irtviewer* :draw-objects)
;
; ;; ちょっと上に手を上げる
; (send *fetch* :angle-vector #f(298.121 73.6483 -3.33862 65.7129 -56.857 101.528 50.9987 -40.364 0.169935 13.9634))
; (send *irtviewer* :draw-objects)
;
; ;; つぎに、手を前に出して、（袋のなかの想定）、ものを放す
; (send *fetch* :angle-vector #f(298.113 35.0643 -4.32739 65.9107 -56.835 95.2437 -108.435 -40.2981 0.249234 13.9195) )
; (send *irtviewer* :draw-objects)
; (send *fetch* :go-grasp :pos 0)
; (send *irtviewer* :draw-objects)
; (send *fetch* :reset-pose)

;; まず、手を前に出して、ものをつかませる(send *ri* :go-grasp :pos 0)
(send *ri* :speak-jp "これから動きます")

(send *ri* :start-grasp)

(send *ri* :speak-jp "ものをつかみにかかります")

(send *ri* :wait-interpolation)



(send *ri* :angle-vector #f(45.2426 57.9818 3.49487 77.8199 -59.6255 10.2316 -78.2444 74.1795 0.27191 15.2598) 25000)

(send *ri* :wait-interpolation)
; (send *irtviewer* :draw-objects)
(send *ri* :stop-grasp)
; (send *irtviewer* :draw-objects)

;; ちょっと上に手を上げる
; (send *ri* :angle-vector #f(298.121 73.6483 -3.33862 65.7129 -56.857 101.528 50.9987 -40.364 0.169935 13.9634) 25000)
; (send *irtviewer* :draw-objects)
(send *ri* :speak-jp "もちあげます")

;; つぎに、手を前に出して、（袋のなかの想定）、ものを放す
(send *ri* :angle-vector #f(298.113 35.0643 -4.32739 65.9107 -56.835 95.2437 -108.435 -40.2981 0.249234 13.9195) 25000)

(send *ri* :wait-interpolation)
; (send *irtviewer* :draw-objects)
(send *ri* :start-grasp)
; (send *irtviewer* :draw-objects)
; (send *ri* :reset-pose)
