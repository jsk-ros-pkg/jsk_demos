;; todo ゼミ11/16
;; がぞうにんしきのやり方とかっこいいシミュレーションのやり方を教わる。


;; ファイルをもたせたい
;; ファイルを作る

(when (not (boundp '*pr2*))
  (require "package://pr2eus/pr2-utils.l")
  (setq *pr2* (pr2)))

(setq file (make-cube 265 210 15))
(send file :set-color :green)

(setq file-rhandle (instance cascaded-coords :init :rot #2f((0 0 1) (1 0 0) (0 1 0))))
(send file-rhandle :locate #f(-100 -105 0) :world)
(send file :assoc file-rhandle)

(setq file-lhandle (instance cascaded-coords :init :rot #2f((0 0 1) (-1 0 0) (0 -1 0))))
(send file-lhandle :locate #f(-100 105 0) :world)
(send file :assoc file-lhandle)

(send file :locate #f(700 0 800))

(setq rhand (send *pr2* :rarm :end-coords))
(setq lhand (send *pr2* :larm :end-coords))

;; 使いません
(defun rarm-catchfile () 
  (send *pr2* :rarm :inverse-kinematics
	(send file-rhandle :worldcoords)
	:link-list (send *pr2* :link-list (send *pr2* :rarm :end-coords :parent))
	:move-target (send *pr2* :rarm :end-coords)
	:rotation-axis t
	:debug-view nil)
  )

(defun pr2-ik-both (ltarget rtarget)
  (send *pr2* :inverse-kinematics
	(list ltarget rtarget)
	:move-target (list (send *pr2* :larm :end-coords)
			   (send *pr2* :rarm :end-coords))
	:use-torso t
	:use-base nil
	)
  )

(defun pr2-ik (isL target)
  (if isL
      (send *pr2* :inverse-kinematics
	    target
	    :move-target (send *pr2* :larm :end-coords)
	    :use-torso nil
	    )
    (send *pr2* :inverse-kinematics
	  target
	  :move-target (send *pr2* :rarm :end-coords)
	  :use-torso nil
	  ))
  )

(defun pr2-ik-l (target) (pr2-ik t target))
(defun pr2-ik-r (target) (pr2-ik nil target))

;; ファイルを受け取る姿勢
(defun catch-file ()
  (pr2-ik-both (send file-lhandle :worldcoords) (send file-rhandle :worldcoords))
  )

;; 右手をはなす
(defun rhand-leave ()
  (setq rhand-target (send file-rhandle :copy-worldcoords))
  (send rhand-target :translate #f(-200 0 0) rhand-target)
  (pr2-ik-r rhand-target))

;; 左手を傾ける
(defun lhand-tilt ()
  (send lhand :assoc file)
  (setq lhand-target (send file-lhandle :copy-worldcoords))
  (send lhand-target :rotate -0.5 :z lhand-target)
  (pr2-ik-l lhand-target))

;; 右手を戻す
(defun rhand-catch-again ()
  (setq rhand-target (send rhand :copy-worldcoords))
  (send rhand-target :translate #f(200 0 0) rhand-target)
  (pr2-ik-r rhand-target))

;; 左手を離す、右手を右側にやる
(defun lhand-leave ()
  (send file :dissoc file-rhandle) ;; 右手で持つ
  (send file-rhandle :assoc file)
  (send file-rhandle :newcoords (send rhand :worldrot) (send rhand :worldpos))
  (send file-rhandle :dissoc file) (send file :assoc file-rhandle)
  (send rhand :assoc file)
  (setq rhand-target (send file-rhandle :copy-worldcoords))
  (send rhand-target :translate #f(-100 0 0) rhand-target)
  (pr2-ik-r rhand-target)
  )

;; ファイルを開く
(defun open-file ()
  (setq rhand-target (send file-rhandle :copy-worldcoords))
  (setq rmat (make-coords))
  (send rmat :orient -0.05 :z)
  (dotimes (i 20)
    ;;    (send rhand-target :rotate -0.05 :z lhand-target)
    (send rhand-target :transform rmat lhand-target) 
    (pr2-ik-r rhand-target)
    (send *irtviewer* :draw-objects)
    (unix:usleep 100000)
    )
  (setq rmat2 (make-coords))
  (send rmat2 :orient 0.05 :z)
  (dotimes (i 15)
    (send rhand-target :transform rmat2 lhand-target) 
    (pr2-ik-r rhand-target)
    (send *irtviewer* :draw-objects)
    (unix:usleep 100000) 
    )
  )

(defun all-moves ()
  (if (send file :parent) (send (send file :parent) :dissoc file))
  (send file :locate #f(500 0 800) :world)
  (send *pr2* :reset-pose)
  (send *irtviewer* :draw-objects)
  (catch-file)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (rhand-leave)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (lhand-tilt)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (rhand-catch-again)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (lhand-leave)
  (unix:sleep 1) (send *irtviewer* :draw-objects)
  (open-file)
  )

 (objects (list file file-rhandle file-lhandle *pr2* rhand lhand))
