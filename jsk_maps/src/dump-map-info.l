(defun unresolve-ros-path (p)
  (let ((pkg-path (ros::resolve-ros-path "package://jsk_maps/")))
    (when (and
           (>= (length p) (length pkg-path))
           (string= (subseq p 0 (length pkg-path)) pkg-path))
      (concatenate string "$(find jsk_maps)/" (subseq p (length pkg-path) (length p))))))

(defun dump-map-image-info (nav-scene)
  (let* ((cube (send nav-scene :map))
         (face (find '(:top) (send cube :faces)
                     :key #'(lambda(x)(send x :id)) :test #'equal))
         teximg filename size cx cy)
    (setq teximg (send face :get :gl-textureimage))
    (setq filename (send teximg :name))
    (setq size (float-vector (send teximg :width) (send teximg :height)))
    (setq cx (- (elt (send cube :centroid) 0) (/ (* (elt size 0) 50) 2)))
    (setq cy (- (elt (send cube :centroid) 1) (/ (* (elt size 1) 50) 2)))
    (list filename cx cy) ))

(defun get-yaml-filename (nav-scene &key (unresolve) (keepout nil))
  (let ((path (pathname (elt (dump-map-image-info nav-scene) 0)))
        yaml-path)
    (setq yaml-path
          (if keepout
            (format nil "~a~a_keepout.yaml" (send path :directory-string) (send path :name))
            (format nil "~a~a.yaml" (send path :directory-string) (send path :name))))
    (if unresolve
        (unresolve-ros-path yaml-path) yaml-path)))

;; output:
;;   image: fullpath
;;   resolution: 0.050
;;   origin: [x, y, 0]
;;   negate: 0
;;   occupied_thresh: 0.65
;;   free_thresh: 0.192
;;   frame_id: scene-name
(defun dump-map-yaml (nav-scene &key filename (keepout nil))
  (let ((info (dump-map-image-info nav-scene)) imgfile)
    (setq filename (or filename (get-yaml-filename nav-scene :keepout keepout)))
    (with-open-file
        (fd filename :direction :output)
      ;; image filename need to be relative to the yaml filename
      (setq imgfile
            (if keepout
              (format nil "~a_keepout.~a" (send (pathname (elt info 0)) :name) (send (pathname (elt info 0)) :type))
              (format nil "~a.~a" (send (pathname (elt info 0)) :name) (send (pathname (elt info 0)) :type))))
      (format fd "image: ~a
resolution: 0.050
origin: [~a, ~a, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.192
map_id: ~a"
              imgfile
              (/ (elt info 1) 1000.0)
              (/ (elt info 2) 1000.0)
              (send nav-scene :name))
      )
    filename ))

(defun makedirs (path)
  (let ((dirs (send (pathname path) :directory))
        (cur-dir "."))
    (dolist (dir dirs)
      (setq cur-dir (concatenate string cur-dir "/" dir))
      (case (probe-file cur-dir)
        (:directory t)
        (nil
         (unless (unix:mkdir cur-dir)
           (error "failed to make directory ~A (error code: ~A)" cur-dir (unix:mkdir cur-dir))))
        (t (error "failed to make directory ~A (probe-file: ~A)" cur-dir (probe-file cur-dir)))))
    t))

(defun dump-map-launch (building filename &key default-floor (keepout nil))
  (let ((floors (send building :floors))
        (cntr 0))
    (if (stringp default-floor)
        (setq default-floor
              (find default-floor floors :test #'string= :key #'(lambda(x)(send x :name)))))
    (if default-floor
        (setq floors (append (list default-floor) (remove default-floor floors))))
    ;;
    (dolist (map floors)
      (dump-map-yaml map)
      (when (equal (send building :name) "/eng2")
        (dump-map-yaml map :keepout t)))

    (makedirs filename)
    ;;
    (with-open-file
        (fd filename :direction :output)
      (format fd "<launch>~%")
      ;; argument
      (when keepout (format fd "  <arg name=\"keepout\" default=\"true\"/>~%"))
      (format fd "  <arg name=\"launch_map_server\" default=\"true\"/>~%~%")

      ;; group
      (format fd "  <group if=\"$(arg launch_map_server)\">~%")
      (format fd "    <!-- map for visualization nodes -->~%")

      ;; map servers
      (if keepout
        (progn
          (format fd "    <group if=\"$(arg keepout)\">~%")
          (dolist (map floors)
            (format fd "      <node name=\"$(anon map_server~2,2D)\" pkg=\"map_server\" type=\"map_server\"
            args=\"~a\" >
        <remap from=\"map\" to=\"~a\"/>
        <param name=\"frame_id\" value=\"~a\"/>
      </node>~%"
            cntr (get-yaml-filename map :unresolve t :keepout t)
            (send map :name) (send map :name))
          (incf cntr))
          (format fd "    </group>~%")))

      (setq cntr 0)
      (if keepout
        (progn
          (format fd "    <group unless=\"$(arg keepout)\">~%")
          (dolist (map floors)
            (format fd "      <node name=\"$(anon map_server~2,2D)\" pkg=\"map_server\" type=\"map_server\"
            args=\"~a\" >
        <remap from=\"map\" to=\"~a\"/>
        <param name=\"frame_id\" value=\"~a\"/>
      </node>~%"
              cntr (get-yaml-filename map :unresolve t :keepout nil)
              (send map :name) (send map :name))
            (incf cntr))
          (format fd "    </group>~%"))
        (dolist (map floors)
          (format fd "    <node name=\"$(anon map_server~2,2D)\" pkg=\"map_server\" type=\"map_server\"
          args=\"~a\" >
      <remap from=\"map\" to=\"~a\"/>
      <param name=\"frame_id\" value=\"~a\"/>
    </node>~%"
            cntr (get-yaml-filename map :unresolve t)
            (send map :name) (send map :name))
          (incf cntr)))

      ;; multi map server
      (format fd "~%    <!-- map for navigation nodes -->~%")
      (if keepout
        (progn
          (format fd "    <node name=\"map_server\" pkg=\"multi_map_server\" type=\"multi_map_server\"
          if=\"$(arg keepout)\" args=\"~a~%"
            (get-yaml-filename (car floors) :unresolve t :keepout t))
          (dolist (map (cdr floors))
            (format fd "          ~a~%" (get-yaml-filename map :unresolve t :keepout t)))
          (format fd "    \" />~%")
          (format fd "    <node name=\"map_server\" pkg=\"multi_map_server\" type=\"multi_map_server\"
          unless=\"$(arg keepout)\" args=\"~a~%"
            (get-yaml-filename (car floors) :unresolve t :keepout nil))
          (dolist (map (cdr floors))
            (format fd "          ~a~%" (get-yaml-filename map :unresolve t :keepout nil)))
          (format fd "    \" />~%"))
        (progn
          (format fd "    <node name=\"map_server\" pkg=\"multi_map_server\" type=\"multi_map_server\"
          args=\"~a~%"
            (get-yaml-filename (car floors) :unresolve t :keepout nil))
          (dolist (map (cdr floors))
            (format fd "          ~a~%" (get-yaml-filename map :unresolve t :keepout nil)))
          (format fd "    \" />~%")))

      ;; group end
      (format fd "  </group>~%~%")

      ;; static transform publisher
      (setq cntr 0)
      (dolist (map floors)
        (format fd "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"$(anon map_tf~2,2D)\"
        args=\"~7,3f ~7,3f ~7,3f 0 0 0 ~a ~a 100\"/>~%"
                cntr
                (/ (elt (send map :worldpos) 0) 1000.0)
                (/ (elt (send map :worldpos) 1) 1000.0)
                (/ (elt (send map :worldpos) 2) 1000.0)
                (send building :name) (send map :name))
        (incf cntr)
        (format fd "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"$(anon map_tf~2,2D)\"
        args=\"  0.000   0.000   0.000 0 0 0 ~a /map 100\" >~%
    <remap from=\"tf\" to=\"~a_tf\"/>
  </node>~%"
                cntr (send map :name) (send map :name))
        (incf cntr)
        )
      ;;
      (format fd "  <node name=\"tf_mux\" pkg=\"topic_tools\" type=\"mux\"
        args=\"tf ")
      (dolist (map floors)
        (format fd " ~a_tf" (send map :name)))
      (format fd "\">
    <remap from=\"mux\" to=\"map_tf_mux\" />
  </node>~%")
      ;;
      (format fd "</launch>~%")
      )))


(defun dump-all-maps ()
  (load "eng2-scene.l")
  (dump-map-launch (make-eng2-scene) "launch/include/start_map_eng2_main.xml"
                   :default-floor "/eng2/7f" :keepout t)
  (load "eng8-scene.l")
  (dump-map-launch (make-eng8-scene) "launch/include/start_map_eng8_main.xml"
                   :default-floor "/eng8/6f")
  (load "eng6-scene.l")
  (dump-map-launch (make-eng6-scene) "launch/include/start_map_eng6_main.xml")
  )

(progn
  (dump-all-maps)
  (exit))
