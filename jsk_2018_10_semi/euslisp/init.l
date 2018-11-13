(load "package://fetcheus/fetch-interface.l")
(load "package://jsk_maps/src/eng2-scene.l")

(defun init ()
  (fetch-init)
  (if (not (boundp '*eng2-scene*))
      (setq *eng2-scene* (make-eng2-scene)))
  (setq *7f* (car (member-if #'(lambda (x) (string= (send x :name) "/eng2/7f")) (send *eng2-scene* :floors))))
  (objects (list *fetch* *7f*))
  (send *fetch* :move-to (send (send *7f* :spot "/eng2/7f/room73B2-table-side1") :copy-worldcoords))
  (send *irtviewer* :look-all (list *fetch*))
  )


 
