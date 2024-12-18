#!/usr/bin/env roseus
;; PR2のモデルを読み込む
(require "package://pr2eus/pr2.l")
(require "package://pr2eus/pr2-utils.l")
(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/speak.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(pr2-init)
;; PR2のインスタンスを作成
(if (not (boundp '*pr2*)) (setq *pr2* (pr2)))

;; (defun make-target-model (target)
;;   (if (= target "clothes")
;;       (make-clothes)
;;     )
;;   (if (= target "person")
;;       (make-person)
;;     )
;;   )

;; 直方体を作成（かかっている服の形状を模擬）
(defun make-clothes ()
  (let* ((width-x 400) ;; 上辺の幅
         (height-top-z 100) ;; 服からハンガー部分までの高さ
         (thickness-y (if (boundp '*target-dimensions*)
                          (* 1000 (send *target-dimensions* :y))
                        40)) ;;厚さ
         (width-bottom-x (if (boundp '*target-dimensions*)
                          (* 1000 (send *target-dimensions* :x))
                        400));;服の認識された幅
         (height-bottom-z (if(boundp '*target-dimensions*)
                              (* 1000 (send *target-dimensions* :z))
                            600)) ;; 服の認識された高さ

        ;  (clothes (make-prism
        ;            (list (float-vector (- (/ width-bottom-x 2)) 0 (/ height-bottom-z 2))       ;; 左上の頂点
        ;                  (float-vector (/ width-bottom-x 2) 0 (/ height-bottom-z 2))    ;; 右上の頂点
        ;                  ;;(float-vector (/ width-x 2) 0 (+ height-top-z (/ height-bottom-z 2))) ;;中央部頂点
        ;                  (float-vector (- (/ width-bottom-x 2)) 0 (- (/ height-bottom-z 2))) ;; 下手前頂点
        ;                  (float-vector (/ width-bottom-x 2) 0 (- (/ height-bottom-z 2))) ;; 下奥頂点
        ;                  ) 
        ;            thickness-y)))  ;; 厚さ

        (clothes (make-cube width-bottom-x thickness-y height-bottom-z))
        )

    ;; 直方体の下の角に座標系を設定（把持の手前の位置）
    (send clothes :put :left-coords
          (make-cascoords
           :coords (send (send clothes :copy-worldcoords)
                    :translate (float-vector (- 80 (/ width-bottom-x 2)) 0 (- (/ height-bottom-z 2) 85))  ;; ハンガーの下端から相対位置
                    ;;:translate (float-vector -150 -20 250)
                    )
           :parent clothes))
    
    (when (boundp '*target-coords*)
      (send clothes :move-to *target-coords* :world)
      
      clothes))
  )

;; 初期位置に配置
;;(send *clothes* :translate (float-vector 800 100 1100))

(defun step1-init()
  (send *ri* :speak-jp "今日は何を着たい?")
  (send *pr2* :reset-pose)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :collar-y :joint-angle 0)
  (send *pr2* :larm :shoulder-p :joint-angle 30)
  (send *pr2* :larm :shoulder-r :joint-angle 0)
  (send *pr2* :larm :elbow-p :joint-angle -120)
  (send *pr2* :larm :elbow-r :joint-angle 180)
  (send *pr2* :larm :wrist-p :joint-angle -30)
  (send *pr2* :larm :wrist-r :joint-angle 180)
  (send *pr2* :larm :gripper :joint-angle 40)
  (send *pr2* :head :neck-p :joint-angle 20)
  (send *irtviewer* :draw-objects)


                                        ; これ書けばロボットが動く
  (send *ri* :stop-grasp :arms)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (send *ri* :speak-jp "準備完了しました")
  (send *ri* :wait-interpolation)
  )

(defun step2-approach ()
 (unix:sleep 2)
  (send *ri* :speak-jp "服に腕を伸ばすよ。")
  
  (labels ((try-approach ()
                         (setq ik-ret (send *pr2* :larm :inverse-kinematics
                                            (send (send *clothes* :get :left-coords) :copy-worldcoords)
                                            :rotation-axis nil
                                            :debug-view nil
                                            :use-torso t
                                            ))

                         (if ik-ret ;; ik solved
                             (progn
                               (format t "IK succeeded: ~A~%" ik-ret)
                               (send *pr2* :larm :inverse-kinematics
                                                  (send (send (send *clothes* :get :left-coords) :copy-worldcoords)
                                                        :translate (float-vector -100 0 0))
                                                  :rotation-axis nil
                                                  :debug-view nil
                                                  :use-torso t
                                                  )

                               (send *ri* :angle-vector (send *pr2* :angle-vector) 3000) 
                               (send *ri* :wait-interpolation)
                               (send *pr2* :larm :inverse-kinematics
                                                  (send (send *clothes* :get :left-coords) :copy-worldcoords)
                                                  :rotation-axis nil
                                                  :debug-view nil
                                                  :use-torso t
                                                  )
                               
                               (send *ri* :angle-vector (send *pr2* :angle-vector) 2000) 
                              
                               (send *ri* :wait-interpolation)
                               (unix:sleep 3)
                               t)  ;; 成功を返す
                           
                           ;; (progn ;;ik failed
                           ;;   (setq ik-ret-torso (send *pr2* :larm :inverse-kinematics
                           ;;                            (send (send *clothes* :get :left-coords) :copy-worldcoords)
                           ;;                            :rotation-axis nil
                           ;;                            :use-torso t ;; 腰を使う
                           ;;                            :debug-view nil))
                           ;;   (if ik-ret-torso ;;腰を使うときのikが成功
                           ;;       (progn
                           ;;         (format t "IK succeeded with torso: ~A~%" ik-ret-torso)
                           ;;         (send *ri* :angle-vector (send *pr2* :angle-vector) 3000) 
                           ;;         (send *ri* :wait-interpolation)
                           ;;         (unix:sleep 3)
                           ;;         t)  ;; 成功を返す
                           nil)))
          
          ;;))
          ;; 失敗を返す

          ;; 最大試行回数を設定
          (let ((max-attempts 5)
                (attempt-count 0))
            (while (not (or (try-approach) (>= attempt-count max-attempts)))
              (progn
                (incf attempt-count)
                (let* ((target-pos (send (send *clothes* :get :left-coords) :worldpos))
                       (robot-pos (send *pr2* :worldpos))
                       (diff-x (- (elt target-pos 0) (elt robot-pos 0)))
                       (diff-y (- (elt target-pos 1) (elt robot-pos 1))))
                  
                  (ros::ros-warn "Moving closer to target (attempt ~A/~A)" attempt-count max-attempts)
                  (send *ri* :speak-jp "服に近づくよ。")
                  (send *ri* :go-pos
                        (if (> diff-x 0) 0.1 -0.1)
                        (if (> diff-y 0) 0.1 -0.1)
                        0)
                  (send *ri* :wait-interpolation)
                  (ros::ros-info "start searching clothes")
                  (send *ri* :speak-jp "服を探します。")
                  ;;サブスクライブ
                  ;; (setq box-sync (instance box-label-synchronizer :init
                  ;;                          (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
                  ;;                                (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))
                  ;; (setq *found* nil)
                  ;; (do-until-key
                  ;;  (when *found*
                  ;;    (return))
                  ;;  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
                  ;;  (send *irtviewer* :draw-objects)
                  ;;  (x::window-main-one)
                  ;;  (ros::spin-once))
                  (find-main '("bathrobe" "swearshirt"))
                  
                  )
                )
              )  ;; 移動後少し待機
            
            
            ;; 最終結果の確認
            (if (>= attempt-count max-attempts)
                (send *ri* :speak-jp "服に届きませんでした。")
              (send *ri* :speak-jp "服に到達しました。")
              )
            )
          )
  )


(defun step3-grasp ()
  (send *ri* :stop-grasp :larm :wait t)
  ;;"ハンドを閉じる動作（シミュレーション用）"
  (send *ri* :wait-interpolation)

  (send *ri* :speak-jp "服を掴むよ。")

  ;;(send *pr2* :larm :gripper :joint-angle 0)
  (send *ri* :start-grasp :larm :wait t)
  (send *irtviewer* :draw-objects)
  (unix:sleep 1)
  
  )



(defun step4-lift ()
  (send *ri* :speak-jp "持ち上げるよ。")
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *clothes* :get :left-coords) :copy-worldcoords)
               :translate (float-vector -10 0 -150))
         :rotation-axis t
         :debug-view nil
         :use-torso t
         )
   (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
   (send *ri* :wait-interpolation)
   (send *pr2* :rarm :inverse-kinematics
         (send (send (send *clothes* :get :left-coords) :copy-worldcoords)
               :translate (float-vector 40 0 -100))
         :rotation-axis t
         :debug-view nil
         :use-torso nil
         )
   (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)

   (send *ri* :wait-interpolation)


  (send *pr2*
        ;;:larm
        :inverse-kinematics (send (send (send (send *clothes* :get :left-coords) :copy-worldcoords)
                                        :translate (float-vector 0 0 150))
                                  :rotate (deg2rad -30) :y
                                  )
        :rotation-axis t
        :use-torso t
        :debug-view nil
        )
  
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  )

(defun step5-pull ()
  (send *ri* :speak-jp "手元に引くよ。")
  ;;"手元に引く"
  (send *pr2*
        :larm
        :inverse-kinematics (send (send (send *clothes* :get :left-coords) :copy-worldcoords)
                                  :translate (float-vector -200 0 0))  ;; さらに30cm手前
                                        ;:move-target (send *clothes* :get :left-coords)
                                        ;:link-list (send *pr2* :link-list (send (send *pr2* :larm :end-coords) :parent))
        :rotation-axis nil
        :debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  (send *irtviewer* :draw-objects))

(defun step6-setpose ()
  (send *pr2* :reset-pose)
  (send *pr2* :larm :collar-y :joint-angle 0)
  (send *pr2* :larm :shoulder-p :joint-angle 30)
  (send *pr2* :larm :shoulder-r :joint-angle 0)
  (send *pr2* :larm :elbow-p :joint-angle -120)
  (send *pr2* :larm :elbow-r :joint-angle 180)
  (send *pr2* :larm :wrist-p :joint-angle -30)
  (send *pr2* :larm :wrist-r :joint-angle 180)
  (send *pr2* :larm :gripper :joint-angle 40)
  (send *pr2* :head :neck-p :joint-angle 20)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (send *irtviewer* :draw-objects)
  )

(defun release ()
  (send *ri* :speak-jp "服を離すよ。")
  ;;"ハンドを開く動作（シミュレーション用）"
  ;;(send *pr2* :larm :gripper :joint-angle 40)

  ;; シミュレーションで服の関連付けを解除
  (send *pr2* :larm :end-coords :dissoc *clothes*)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :larm)
  (send *irtviewer* :draw-objects)
  )

                                        ;服を把持しようとしているかどうかのフラグ
(setq *found* nil)
(setq *is-moving* nil)

;; コールバック関数
;;検知したboxとラベルを対応させるクラス
(defclass box-label-synchronizer
  :super exact-time-message-filter
  :slots (target-label-list)
  )

(defmethod box-label-synchronizer
  (:callback (box-msg label-msg)
             (print (list box-msg label-msg))
             (print (send-all (list box-msg label-msg) :header :stamp))
             (box-cb box-msg label-msg target-label-list)
             )
  )

(defmethod box-label-synchronizer
  (:set-target-label-list (label-list)
                          (setq target-label-list label-list))
   )

;; (defun clothes-box-cb (msg)

;;   (dolist (box (send msg :boxes))
;;     (when (member (send box :label) '(566 615 469 923))
;;       (ros::ros-info "hook is detected")
;;       ;(send *ri* :speak-jp "服発見！")
;;       (setq *target-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
;;       (setq *target-dimensions* (send box :dimensions))

;;コールバック関数
;;/docker/detic_segmentor/detected_classesというトピックにidの対応あり


(defun box-cb (box-msg label-msg label-list)
  (ros::ros-info "received ~A boxes, ~A labels" (length (send box-msg :boxes)) (length (send label-msg :labels)))
  (dolist (msg-conbined (map cons #'(lambda (x y) (list  x y)) (send box-msg :boxes) (send label-msg :labels)))
    (let (box label)
      ;;(print (list msg-conbined))                                                                                                                                             
      (setq box (car msg-conbined) label (cadr msg-conbined))
      ;;(print (list box label))                                                                                                                                                
      (print (send label :name))
      ;; (when (or (string= (send label :name) "bathrobe")
      ;;           ;;(string= (send label :name) "jacket")
      ;;           ;;(string= (send label :name) "coat")
      ;;           ;;(string= (send label :name) "shirt")
      ;;           ;;(string= (send label :name) "robe")
      ;;           (string= (send label :name) "sweatshirt")
      ;;           )
      (when (contains label-list (send label :name))
      (setq *target-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
        (setq *target-dimensions* (send box :dimensions))
        (format t "target:coords ~A, dimension ~A~%" (send *target-coords* :worldcoords) (* (send *target-dimensions* :x) (send *target-dimensions* :y) (send *target-dimensions* :z)))
        ;; 座標を表示
        (format t "Target coords: x=~A y=~A z=~A~%"
                (elt (send *target-coords* :worldpos) 0)
                (elt (send *target-coords* :worldpos) 1)
                (elt (send *target-coords* :worldpos) 2))
        
        (when (and (< (elt (send *target-coords* :worldpos) 2) 1800)
                   (> (elt (send *target-coords* :worldpos) 2) 0)
                   ;; (> (elt (send *target-coords* :worldpos) 0) 0)
                   ;; (< (elt (send *target-coords* :worldpos) 0) 1200)
                   ;; (> (elt (send *target-coords* :worldpos) 1) -1500)
                   ;; (< (elt (send *target-coords* :worldpos) 1) 1500)
                   )
          (ros::ros-info "target is in reachable area")

          ;; 服の位置を更新して確認
          (setq *clothes* (make-clothes))
          (objects (append (list *pr2*) (list *clothes*)))
          (send *irtviewer* :draw-objects)
          ;; (send *clothes* :move-to *target-coords* :world)
          ;; (format t "Updated clothes position: ~A~%" (send *clothes* :worldpos))
          ;; (send *irtviewer* :draw-objects)
          ;; (unix:sleep 1)  ;; 位置更新を確認するための待機
          (setq *found* t))
        )
      ))  ;; 処理が完了したらループを抜ける
  );; defun

(defun contains (list word)
  (let ((tmp nil))
    (while list
      (if (string= word (car list))
          (progn (setq tmp t) (return))
        (setq list (cdr list))))
    tmp)
  )


(defun find-main (label-list)
  (setq *found* nil)
  
  ;; (ros::subscribe "/synchronized_detic_label_boxes"
  ;;               jsk_recognition_msgs::BoundingBoxArray #'clothes-box-cb)
  ;;サブスクライブ
  (setq box-sync (instance box-label-synchronizer :init
                           (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
                                 (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))
  (send box-sync :set-target-label-list label-list)
  
  (do-until-key
   (when *found*
     (return))
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (send *irtviewer* :draw-objects)
   (x::window-main-one)
   (ros::spin-once))
  )

;; 服を掴む一連の動作
(defun grasp-clothes ()
  (setq *is-moving* t)
  (unwind-protect

      (progn
        (setq *grasp-yet-tag* t)

        (while *grasp-yet-tag*
          (ros::ros-info "start searching clothes")
          (send *ri* :speak-jp "服を探します。")
          (find-main '("bathrobe" "swearshirt"))
          
            (when *found* ;;服が見つかった時
              (step2-approach)
              (step3-grasp)
          
              (let ((gripper-distance (send *ri* :robot :larm :gripper :joint-angle)))
                (ros::ros-info "gripper-distance ~A mm" gripper-distance)
                (if (< gripper-distance 6.5) ;;6.5mmより指の間が小さいときはやり直し
                    (progn
                      (send *ri* :speak-jp "服を掴めていない気がするよ")
                      (send *ri* :wait-interpolation)
                      (send *ri* :stop-grasp :larm :wait t)
                      )
                (progn
                  (setq *grasp-yet-tag* nil)
                  (send *pr2* :larm :end-coords :assoc *clothes*)
                  )
                )
                )
              )
            )
        

        (unix:sleep 2)
        (step4-lift)
        (step5-pull)
        (unix:sleep 2)
        (step6-setpose)
        )
    
    (setq *is-moving* nil)))


(defun person-box-cb (box-msg label-msg)
  (ros::ros-info "received ~A boxes, ~A labels" (length (send box-msg :boxes)) (length (send label-msg :labels)))
  (dolist (msg-conbined (map cons #'(lambda (x y) (list  x y)) (send box-msg :boxes) (send label-msg :labels)))
    (let (box label)
      ;;(print (list msg-conbined)) 
      (setq box (car msg-conbined) label (cadr msg-conbined))
      ;;(print (list box label))
      (print (send label :name))
      (when (string= (send label :name) "person")
        (setq *person-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
        (setq *person-dimensions* (send box :dimensions))
        (format t "person:coords ~A, dimension ~A~%" (send *person-coords* :worldcoords) (* (send *person-dimensions* :x) (send *person-dimensions* :y) (send *target-dimensions* :z)))
        ;; 座標を表示
        (format t "person coords: x=~A y=~A z=~A~%"
                (elt (send *person-coords* :worldpos) 0)
                (elt (send *person-coords* :worldpos) 1)
                (elt (send *person-coords* :worldpos) 2))
        
        ;;(when (and (< (elt (send *person-coords* :worldpos) 2) 1800)
        ;;           (> (elt (send *person-coords* :worldpos) 2) 0)
                   ;; (> (elt (send *person-coords* :worldpos) 0) 0)
                   ;; (< (elt (send *person-coords* :worldpos) 0) 1200)
                   ;; (> (elt (send *person-coords* :worldpos) 1) -1500)
                   ;; (< (elt (send *person-coords* :worldpos) 1) 1500)
          ;;         )
          ;;(ros::ros-info "person is in reachable area")

          ;; 人の位置を更新して確認
          (setq *person* (make-person))
          (objects (append (list *pr2*) (list *person*)))
          (send *irtviewer* :draw-objects)
          ;; (send *person* :move-to *target-coords* :world)
          ;; (format t "Updated person position: ~A~%" (send *person* :worldpos))
          ;; (send *irtviewer* :draw-objects)
          ;; (unix:sleep 1)  ;; 位置更新を確認するための待機
          (setq *found-person* t))
        ;;)
      ))  ;; 処理が完了したらループを抜ける
  );; defun

(defun make-person ()
  (let ((person (make-cube 200 200 200)))
    (when (boundp '*person-coords*)
      (send person :move-to *person-coords* :world)
      person)))

(defun move-to-person ()
  (send *ri* :speak-jp "人に近づくよ。")
  (let* (person-pos (send *person-coords* :worldpos))
        (robot-pos (send *pr2* :worldpos))
        (diff-x (- (elt person-pos 0) (elt robot-pos 0)))
        (diff-y (- (elt person-pos 1) (elt robot-pos 1)))
        (distance (sqrt (+ (* diff-x diff-x) (* diff-y diff-y))))
        ;;1.5mの距離まで近づく
        (ratio (/ 1500 distance))
        (target-x (* ratio diff-x))
        (target-y (* ratio diff-y)))

  (send *ri* :go-pos target-x target-y 0)
)

(defun search-person-with-timeout ()
  (let ((start-time (ros::time-now))
        (timeout 5.0))  ;; 5秒のタイムアウト
    (setq *found-person* nil)
    (do-until-key
     (when (or *found-person*
               (> (send (ros::time- (ros::time-now) start-time) :to-sec)
                  timeout))
       (return))
     (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
     (send *irtviewer* :draw-objects)
     (x::window-main-one)
     (ros::spin-once))
    *found-person*))  ;; 人が見つかったかどうかを返す

(defun main ()
  (ros::ros-info "start main loop")
  (objects (list *pr2*))
  (step1-init)
  (unix:sleep 1)

  ;; (ros::ros-info "start searching clothes")
  ;; (send *ri* :speak-jp "服を探します。")
  ;; ;; (ros::subscribe "/synchronized_detic_label_boxes"
  ;; ;;               jsk_recognition_msgs::BoundingBoxArray #'clothes-box-cb)
  ;; ;;サブスクライブ
  ;; (setq box-sync (instance box-label-synchronizer :init
  ;;                          (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
  ;;                                (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))

  ;; (do-until-key
  ;;  (when *found-clothes*
  ;;    (return))
  ;;  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  ;;  (send *irtviewer* :draw-objects)
  ;;  (x::window-main-one)
  ;;  (ros::spin-once))

  (grasp-clothes)

 
  ;;     ;; 人を探すプロセス
  ;;  (send *ri* :speak-jp "人を探します。")
  ;;  (setq *found-person* nil)
  ;;  main関数内の人を探すプロセスの前に追加
  ;;   (setq box-sync (instance box-label-synchronizer :init
  ;;                        (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
  ;;                              (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))
  ;; (if (search-person-with-timeout)
  ;;   (progn
  ;;     (send *ri* :speak-jp "人を見つけました。")
  ;;     (move-to-person))
  ;; (progn ;;人ｇ見つからなかった場合
  ;;   (send *ri* :speak-jp "人を見つけられませんでした。")))
  
  (release)

  (send *ri* :speak-jp "終了しました。")
    
  )

;; メイン関数を実行
(main)
