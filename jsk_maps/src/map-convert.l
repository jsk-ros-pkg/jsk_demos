;; load yaml as alist
(defun load-yaml (filename)
  (let (pipe src ret)
    (setq src (concatenate string "python -c \"import yaml
def parse_sexpr (yobj):
  text = ''
  if type(yobj) == list:
    for obj in yobj:
      text += parse_sexpr(obj)
    text = '(list ' + text + ')'
  elif type(yobj) == dict:
    for k in yobj:
      text += '(cons %s %s)' % (parse_sexpr(k),parse_sexpr(yobj[k]))
    text = '(list ' + text + ')'
  elif type(yobj) == str: text = ' \\\"%s\\\"' % yobj
  elif type(yobj) == float: text = ' %f' % yobj
  elif type(yobj) == int: text = ' %d' % yobj
  elif type(yobj) == bool: text = ' t' if yobj else ' nil'
  return text
if __name__ == '__main__':
  print parse_sexpr(yaml.load(open('" filename "').read()))
  exit(0)
\"" ))
    (setq pipe (piped-fork src))
    (prog1
	(eval (read-from-string (read-line pipe nil nil)))
      (while (read-line pipe nil nil))
      (close pipe))
    ))

(defun get-item (key alist)
  (cdr (assoc key alist :test #'equal)))

(defun get-coords (alist)
  (let ((translation (get-item "translation" alist))
	(rotation (get-item "rotation" alist)))
    (make-coords
     :pos (scale 1000
		 (float-vector (get-item "x" translation)
			       (get-item "y" translation)
			       (get-item "z" translation)))
     :rot (quaternion2matrix
	   (float-vector (get-item "w" rotation)
			 (get-item "x" rotation)
			 (get-item "y" rotation)
			 (get-item "z" rotation))))))

(defun get-map-size (filename)
  (let ((img (read-image-file filename)))
    (when img
      (integer-vector (send img :width) (send img :height)))))

(defun convert-map (filename &optional (output (format nil "~A.l" filename)))
  (let (mapserver-yaml-filename image-filename basename
	map-yaml mapserver-yaml imgsize mapsize)
    (setq basename (send (truename filename) :name))
    ;;
    (setq map-yaml (load-yaml filename))
    ;;
    (setq mapserver-yaml-filename
	  (format nil "~A/~A"
		  (send (truename filename) :directory-string)
		  (get-item "map_setting" map-yaml)))
    (setq mapserver-yaml
	  (load-yaml mapserver-yaml-filename))
    ;;
    (setq image-filename
	  (format nil "~A/~A"
		  (send (truename mapserver-yaml-filename)
			:directory-string)
		  (get-item "image" mapserver-yaml)))
    (setq imgsize (get-map-size image-filename))
    (setq mapsize
	  (scale (* 1000 (get-item "resolution" mapserver-yaml))
		 (coerce imgsize float-vector)))

    ;;
    (with-open-file
     (f output :direction :output)
     (setq origin (mapcar #'(lambda(x)(* x 1000))
			  (get-item "origin" mapserver-yaml)))
     (unless (eq 0.0 (elt origin 2))
       (error "origin[2] is not zero: map coordinate is wrong"))
     (setq floor-yaml (get-item "floor" map-yaml))
     (setq room-yaml (get-item "room" map-yaml))

     (format f ";; This file is Generated from ~A~%" filename)
     (pprint
      ;;
      ;; defun make-foo-map ()
      ;;
      `(defun ,(intern (string-upcase (format nil "make-~A-map" basename))) ()
	 (let (map map-img top-face floors assoc-list)
	   ;; map obj
	   (setq map (make-cube ,(elt mapsize 1) ,(elt mapsize 0) 1))
	   (send map :name ,(get-item "map_frame" map-yaml))
	   (send map :translate-vertices
		 (float-vector
		  ,(+ (/ (elt mapsize 1) 2) (elt origin 1))
		  ,(- 0 (/ (elt mapsize 0) 2) (elt origin 0))
		  0))
	   (send map :rotate-vertices pi/2 :z)
	   (setq top-face (find-if #'(lambda(f)(equal (send f :id) '(:top)))
				   (send map :faces)))
	   (setq map-img (read-image-file ,image-filename))
	   (setf (get map-img :texture-max-width) ,(elt imgsize 0))
	   (setf (get map-img :texture-max-height) ,(elt imgsize 1))
	   (send map :paste-texture-to-face top-face :image map-img)
	   (setq top-face (find-if #'(lambda(f)(equal (send f :id) '(:top)))
				   (send map :faces)))
	   ;;
	   (setq assoc-list
		 (list
		  ,@(mapcar
		    #'(lambda(x)
			`(cons ,(get-item "frame_id" (cdr x)) ,(car x)))
		    (append floor-yaml room-yaml))))
	   ;; definition of floors
	   (setq floors
		 (list
		  ,@(mapcar
		     #'(lambda(x)
			 `(make-cascoords
			   :name ,(car x)
			   :pos ,(concatenate cons '(float-vector) (send (get-coords (cdr x)) :pos))
			   :rpy ,(concatenate cons '(float-vector) (car (rpy-angle (send (get-coords (cdr x)) :rot))))))
		     floor-yaml)))
	   (setq regions
		 (list
		  ,@(mapcar
		     #'(lambda(x)
			 `(make-prism
			   (list ,@(mapcar
				    #'(lambda(y)(list 'float-vector (* 1000 (elt y 0)) (* 1000 (elt y 1)) 0))
				    (get-item "region" (cdr x))))
			   100.0
			   :name ,(format nil "~A/region" (car x))))
		     floor-yaml)))
	   (mapcar #'(lambda(co pr)(send pr :move-to co :local)(send co :assoc pr)) floors regions)
	   (dolist (flo floors)
	     (let ((parent
		    (find (car (rassoc (send flo :name) assoc-list))
			  (append (list map) (flatten (send map :inheritance)))
			  :key #'(lambda(x)(send x :name)) :test #'equal)))
	       (send flo :transform parent :world)
	       (send parent :assoc flo)))
	   ;; definition of rooms
	   (setq rooms
		 (list
		  ,@(mapcar
		     #'(lambda(x)
			 `(make-cascoords
			   :name ,(car x)
			   :pos ,(concatenate cons '(float-vector) (send (get-coords (cdr x)) :pos))
			   :rpy ,(concatenate cons '(float-vector) (car (rpy-angle (send (get-coords (cdr x)) :rot))))))
		     room-yaml)))
	   (dolist (rm rooms)
	     (let ((parent
		    (find (car (rassoc (send rm :name) assoc-list))
			  (append (list map) (flatten (send map :inheritance)))
			  :key #'(lambda(x)(send x :name)) :test #'equal)))
	       (send rm :transform parent :world)
	       (send parent :assoc rm)))
	   ;; end of room definition
	   map))
      f ) ;; end of pprint
     )
    ))
