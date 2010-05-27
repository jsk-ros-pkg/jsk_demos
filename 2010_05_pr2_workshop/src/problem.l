;; define problem
(define-pddl-problem *problem*
  :name pr2-workshop
  :domain pr2
  :objects ((cup . obj)
            (pet . obj)
            (desk . obj)
            (larm . obj)
            (rarm . obj))
  :initial-condition ((on cup desk)
                      (on pet desk)
                      (hand-empty? larm)
                      (hand-empty? rarm)
                      (empty? cup))
  :goal-condition ((on cup desk)
                   (on pet desk)
                   (not (empty? cup))
                   (empty? pet))
  )
;; define domain
(define-pddl-domain *domain*
  :name pr2
  :requirements (:typing)
  :types (object)
  :predicates ((on ?obj0 ?obj1 - object)
               (grasp? ?arm ?obj - object)
               (hand-empty? ?arm - object)
               (empty? ?obj - object))
  :actions
  ((pick
    :parameters ((?arm ?obj ?from object))
    :precondition ((not (grasp? ?arm ?obj))
                   (on ?obj ?from)
                   (hand-empty? ?arm))
    :effect ((not (hand-empty? ?arm))
             (not (on ?obj ?from))
             (grasp? ?arm ?obj)))
   (put
    :parameters ((?arm ?obj ?to object))
    :precondition ((grasp? ?arm ?obj))
    :effect ((hand-empty? ?arm)
             (not (grasp ?arm ?obj))
             (on ?obj ?to)))
   (pour
    :parameters ((?from ?to ?from-arm ?to-arm object))
    :precondition ((grasp? ?from-arm ?from)
                   (grasp? ?to-arm ?to)
                   (empty? ?to)
                   (not (empty? ?from)))
    :effect ((not (empty? ?to))
             (empty? ?from)))
   ))
