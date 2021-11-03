#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_rviz_plugins")

(defvar *scene* nil)
(defvar *current-map-id* nil)
(defvar *pub-spots-marker-topic* "/spots_marker_array")
(defvar *pub-spots-pictogram-topic* "/spots_pictogram")
(defvar *tf-listener* nil)
(defvar *scene-name* nil)
(defvar *use-pictogram-p* nil)

(defun make-pin-marker (name coords header id
                        &key (height 1800) (radius 100) (alpha 1.0) (lifetime 15) color)
  "Make pin marker messages for coordinates.
   if :color is set, pin color is set, otherwise random color is set automatically."
  (let ((cyl (make-cylinder radius (* radius 0.7) :segments 9 :pos (float-vector 0 0 (/ height 3))))
        (cub (make-cube (/ radius 3) (/ radius 3) (/ height 2.5) :pos (float-vector 0 0 (/ height 6))))
        (cds (send coords :copy-worldcoords)))

    ;; transform pin
    (send cyl :rotate -pi/2 :y)
    (send cyl :transform cds :world)
    (send cub :transform cds :world)

    ;; generate random color
    (unless color
      (setq color (scale (/ 1.0 255.0) (apply #'float-vector (color-category20 id)))))

    ;; make pin
    (list
     (cylinder->marker-msg cyl header :color color :ns "pin_head" :lifetime lifetime :id id :alpha alpha)
     (cube->marker-msg cub header :color color :ns "pin_body" :lifetime lifetime :id id :alpha alpha)
     (text->marker-msg name (send cds :translate (float-vector 0 0 (/ height 6)))
                       header :ns "pin_label" :lifetime lifetime :id id))))

(defun publish-pin-marker-array (spots header &key (alpha 0.8) (lifetime 15))
  "Publish marker array from spots.
   'spots' is list of cons (name . coords) for each spots."
  (unless (ros::get-topic-publisher *pub-spots-marker-topic*)
    (ros::advertise *pub-spots-marker-topic* visualization_msgs::MarkerArray 5)
    (unix:sleep 1))
  (let ((id 0))
    (ros::publish
     *pub-spots-marker-topic*
     (instance visualization_msgs::MarkerArray :init
               :markers
               (flatten
                (mapcar #'(lambda (spot)
                            (make-pin-marker (car spot) (cdr spot)
                                             header (incf id)
                                             :alpha alpha
                                             :lifetime lifetime)) spots))))))

(defun make-pictogram (name coords header id
                       &key (size 0.5) (lifetime 15) (alpha 0.8)
                            (color (float-vector 1.0 0.341176 0.133333)))
  "Make pictogram from coords and label marker from name as a list of cons (pic . label)"
  (let ((pictogram (instance jsk_rviz_plugins::Pictogram :init :header header))
        (cds (send coords :copy-worldcoords)))

    (send cds :translate (float-vector 0 0 500) :world)
    (send pictogram :pose (ros::coords->tf-pose cds))

    (let ((c (make-coords)))
      (send c :rotate (deg2rad -90) :y)
      (send c :rotate (deg2rad 180) :z)
      (let ((q (matrix2quaternion (send c :rot))))
        (send pictogram :pose :orientation :w (elt q 0))
        (send pictogram :pose :orientation :x (elt q 1))
        (send pictogram :pose :orientation :y (elt q 2))
        (send pictogram :pose :orientation :z (elt q 3))))

    (send pictogram :size size)
    (send pictogram :character "fa-chevron-circle-up")
    (send pictogram :color :r (elt color 0))
    (send pictogram :color :g (elt color 1))
    (send pictogram :color :b (elt color 2))
    (send pictogram :color :a alpha)


    (send cds :translate (float-vector 0 0 100) :world)
    (cons pictogram
          (text->marker-msg name cds header
                            :ns "pin_label" :lifetime lifetime :id id))))

(defun publish-pictogram-array (spots header
                                &key (alpha 0.8) (lifetime 15) (size 0.5)
                                     (color (float-vector 1.0 0.341176 0.133333)))
  "Publish marker array from spots.
   'spots' is list of cons (name . coords) for each spots."
  (unless (ros::get-topic-publisher *pub-spots-pictogram-topic*)
    (ros::advertise *pub-spots-pictogram-topic* jsk_rviz_plugins::PictogramArray 5)
    (unix:sleep 1))
  (unless (ros::get-topic-publisher *pub-spots-marker-topic*)
    (ros::advertise *pub-spots-marker-topic* visualization_msgs::MarkerArray 5)
    (unix:sleep 1))
  (let ((id 0)
        pic pictograms markers)
    (dolist (spot spots)
      (setq pic (make-pictogram (car spot) (cdr spot) header (incf id)
                                :alpha alpha :lifetime lifetime
                                :size size :color color))
      (push (car pic) pictograms)
      (push (cdr pic) markers))
    (ros::publish *pub-spots-pictogram-topic*
                  (instance jsk_rviz_plugins::PictogramArray :init
                            :header header
                            :pictograms pictograms))
    (ros::publish *pub-spots-marker-topic*
                  (instance visualization_msgs::MarkerArray :init
                            :markers markers))))

