#!/usr/bin/env roseus

;; detect_cans_in_fridge_201202/euslisp/main.l を参考に書いていく

;; set ボウル
(load "cad_bowl.l")
(setq *bowl* (cad_bowl))
(send *bowl* :locate #f(850 100 825))

;; set pot
(load "cad_pot.l")
(setq *cad-pot* (cad_pot))
(send *cad-pot* :locate #f(680 700 830))


;; set otama
(load "cad_otama.l")
(setq *otama* (cad_pot))
(send *otama* :locate #f(900 200 825))


;; robot initialize
(load "package://pr2eus/pr2.l")   
(setq *robot* (pr2)) 

;; set kitchen
(load "models/room73b2-kitchen-object.l")
(setq *kitchen* (room73b2-kitchen))
(send *kitchen* :locate #f(800 -200 0))
(send *kitchen* :rotate (deg2rad 180) :z :world)

;; set knife
(load "models/knife-object.l")
(setq *knife* (knife))
(send *knife* :locate #f(550.0 -200.0 850.0))


;; set まな板
(load "cutting-board.l")
(setq *cutting-board* (cutting-board))
(send *cutting-board* :locate #f(650 0 825))

;; set 調理鍋
(load "cooking-pot.l")
(setq *pot* (cooking-pot))
(send *pot* :locate #f(680 500 830))


(setq *env-list* (list *kitchen* *knife* *cutting-board* *pot* *bowl* *cad-pot* *otama*))

;; (objects (list *bowl* *robot*)) 
(objects (append *env-list* *robot*))


