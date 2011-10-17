(defun dump-map-image-info (nav-scene)
  (let* ((cube (send nav-scene :map))
	 (face (find '(:top) (send cube :faces)
		     :key #'(lambda(x)(send x :id)) :test #'equal))
	 teximg filename size cx cy)
    (setq teximg (send face :get :gl-textureimage))
    (setq filename (send teximg :name))
    (setq size (float-vector (send teximg :width) (send teximg :height)))
    (setq cx (- (elt (send cube :centroid) 0) (/ (* (elt size 1) 50) 2)))
    (setq cy (- (elt (send cube :centroid) 1) (/ (* (elt size 0) 50) 2)))
    (list filename cx cy) ))

(defun get-yaml-filename (nav-scene)
  (let ((path (pathname (elt (dump-map-image-info nav-scene) 0))))
    (setq filename (format nil "~a~a.yaml" (send path :directory-string) (send path :name)))))

;; output:
;;   image: fullpath
;;   resolution: 0.050
;;   origin: [x, y, 0]
;;   negate: 0
;;   occupied_thresh: 0.65
;;   free_thresh: 0.192
;;   frame_id: scene-name
(defun dump-map-yaml (nav-scene &optional filename)
  (let ((info (dump-map-image-info nav-scene)))
    (setq filename (or filename (get-yaml-filename nav-scene)))
    (with-open-file
     (fd filename :direction :output)
     (format fd "image: ~a
resolution: 0.050
origin: [~a, ~a, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.192
map_id: ~a"
	     (elt info 0)
	     (/ (elt info 1) 1000.0)
	     (/ (elt info 2) 1000.0)
	     (send nav-scene :name))
     )
    filename ))


(defun dump-map-launch (building filename)
  ;;
  (dolist (map (send building :floors))
    (dump-map-yaml map))
  ;;
  (with-open-file
   (fd filename :direction :output)
   (format fd "<launch>~%")
   ;;
   (format fd "  <!-- map for visualization nodes -->~%")
   (dolist (map (send building :floors))
     (format fd "  <node name=\"$(anon map_server)\" pkg=\"map_server\" type=\"map_server\"
        args=\"~a\" >
    <remap from=\"map\" to=\"~a\"/>
    <param name=\"frame_id\" value=\"~a\"/>
  </node>~%" (get-yaml-filename map) (send map :name) (send map :name))
     )
   ;;
   (format fd "~%  <!-- map for navigation nodes -->~%")
   (format fd "  <node pkg=\"map_server2\" type=\"map_server\" name=\"map_server\"~%        args=\"")
   (dolist (map (send building :floors))
     (format fd " ~a~%" (get-yaml-filename map)))
   (format fd "\"/>~%")
   ;;
   (dolist (map (send building :floors))
     (format fd "  <node pkg=\"tf\" type=\"static_transform_publisher\"
    name=\"$(anon map_tf)\"
    args=\"~a ~a ~a 0 0 0 ~a ~a 10\"/>~%"
	     (/ (elt (send map :worldpos) 0) 1000.0)
	     (/ (elt (send map :worldpos) 1) 1000.0)
	     (/ (elt (send map :worldpos) 2) 1000.0)
	     (send building :name) (send map :name))
     (format fd "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"$(anon map_tf)\"
        args=\"0 0 0 0 0 0 ~a /map 10\">
    <remap from=\"tf\" to=\"~a_tf\"/>
  </node>~%"
	     (send map :name) (send map :name))
     )
   ;;
   (format fd "  <node name=\"tf_mux\" pkg=\"topic_tools\" type=\"mux\"
        args=\"tf ")
   (dolist (map (send building :floors))
     (format fd " ~a_tf" (send map :name)))
   (format fd "\">
    <remap from=\"mux\" to=\"map_tf_mux\" />
  </node>")
   ;;
   (format fd "</launch>~%")
   ))


(defun dump-all-maps ()
  (load "eng2-scene.l")
  (dump-map-launch (make-eng2-scene) "launch/start_map_eng2_main.launch")
  ;;(load "eng8-scene.l")
  ;;(dump-map-launch (make-eng8-scene) "../launch/start_map_eng8.launch")
  )

(dump-all-maps)