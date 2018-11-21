
;忘れ物した動き
(defun forget nil
  ; 首振り
  (send *fetch* :head :neck-y :joint-angle -45)  
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
 
  (send *fetch* :head :neck-y :joint-angle 45)  
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
 
  (send *fetch* :head :neck-y :joint-angle -45)  
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
 
  (send *fetch* :head :neck-y :joint-angle 0)  
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
 
  ;ちょっと車高を上げて腕を上げる
  (send *fetch* :torso :waist-z :joint-angle 100)  
  (send *fetch* :rarm :shoulder-r :joint-angle -190)           
  (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
 
  ;腕を上げて頭をかきかき
  (send *fetch* :rarm :shoulder-p :joint-angle 5)
  (send *fetch* :rarm :wrist-p :joint-angle 60) 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
 
  (send *fetch* :rarm :shoulder-p :joint-angle -10)
  (send *fetch* :rarm :wrist-p :joint-angle 50) 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
 
  (send *fetch* :rarm :shoulder-p :joint-angle 5)
  (send *fetch* :rarm :wrist-p :joint-angle 60) 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
 
  (send *fetch* :rarm :shoulder-p :joint-angle -10)
  (send *fetch* :rarm :wrist-p :joint-angle 50) 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
 
  (send *fetch* :rarm :shoulder-p :joint-angle 5)
  (send *fetch* :rarm :wrist-p :joint-angle 60) 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
 
  (send *ri* :angle-vector (send *fetch* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
)

;リセットポーズになる
(defun reset nil
  (send *fetch* :reset-pose)
  (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
)

(defun wasuremono.l nil
(setq table_side (send (send *eng2-scene* :spot "/eng2/7f/room73B2-table-side1") :copy-worldcoords))
(setq table_side_1 (send table_side :transform (make-coords :pos #f(-500 0 0) :rpy (float-vector 0 0 0))))

(setq door_front (send (send *eng2-scene* :spot "/eng2/7f/room73A3-front") :copy-worldcoords))
(setq door_front_1 (send door_front :transform (make-coords :pos #f(-300 -300 0) :rpy (float-vector 0 0 0))))

(send *fetch* :move-to table_side_1 :world)
(send *ri* :move-to table_side_1)
(send *ri* :wait-interpolation)

(send *fetch* :move-to door_front_1 :world)
(send *ri* :move-to door_front_1)
(send *ri* :wait-interpolation)

(forget)
(reset)

(send *fetch* :move-to table_side_1 :world)
(send *ri* :move-to table_side_1)
(send *ri* :wait-interpolation)
)
