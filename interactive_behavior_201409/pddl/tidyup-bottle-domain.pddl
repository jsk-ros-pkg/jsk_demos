(define
 (domain room-domain)
 (:requirements :adl :typing :action-costs :negative-preconditions)
 (:types spot arm item person)
 (:constants arms - arm cleaner - item somewhere - spot someone - person)
 (:predicates
  (robot-at ?at - spot)
  (sure-on ?obj - item ?at - spot)
  (on ?obj - item ?at - spot)
  (sure-see ?who - person ?at - spot)
  (see ?who - person ?at - spot)
  (delivered ?obj - item ?who - person)
  (asked ?who - person)
  (onhand ?obj - item ?arm - arm)
  (open ?at - spot)
  (available ?at - spot)
  (placable ?obj - item ?at - spot)
  (openable ?arm - arm ?at - spot)
  (clean))
 (:functions
  (total-cost)
  (distance ?from ?to - spot)
  (manip-cost ?obj - item ?arm - arm)
  (find-cost)
  (door-cost))
 (:action
  "move-to"
  :parameters
  (?from ?to - spot)
  :precondition
  (and (robot-at ?from) (not (open ?from)))
  :effect
  (and
     (not (robot-at ?from))
     (robot-at ?to)
     (forall (?obj - item) (not (sure-on ?obj ?from)))
     (increase (total-cost) (distance ?from ?to)))
  :duration
  nil)
 (:action
  "find-object"
  :parameters
  (?obj - item ?at - spot)
  :precondition
  (and (robot-at ?at) (on ?obj ?at) (available ?at))
  :effect
  (and (sure-on ?obj ?at) (increase (total-cost) (find-cost)))
  :duration
  nil)
 (:action
  "pick"
  :parameters
  (?obj - item ?arm - arm ?at - spot)
  :precondition
  (and
     (forall
      (?other - item)
      (and (not (onhand ?other ?arm)) (not (onhand ?other arms))))
     (robot-at ?at)
     (sure-on ?obj ?at)
     (available ?at))
  :effect
  (and
     (onhand ?obj ?arm)
     (not (sure-on ?obj ?at))
     (not (on ?obj ?at))
     (increase (total-cost) (manip-cost ?obj ?arm)))
  :duration
  nil)
 (:action
  "find-placement"
  :parameters
  (?obj - item ?arm - arm ?at - spot)
  :precondition
  (and (onhand ?obj ?arm) (robot-at ?at) (available ?at))
  :effect
  (and (placable ?obj ?at) (increase (total-cost) (find-cost)))
  :duration
  nil)
 (:action
  "place"
  :parameters
  (?obj - item ?arm - arm ?at - spot)
  :precondition
  (and
     (onhand ?obj ?arm)
     (robot-at ?at)
     (available ?at)
     (placable ?obj ?at))
  :effect
  (and
     (not (onhand ?obj ?arm))
     (on ?obj ?at)
     (not (placable ?obj ?at))
     (increase (total-cost) (manip-cost ?obj ?arm)))
  :duration
  nil)
 (:action
  "open-door"
  :parameters
  (?arm - arm ?at - spot)
  :precondition
  (and
     (forall
      (?obj - item)
      (and (not (onhand ?obj ?arm)) (not (onhand ?obj arms))))
     (robot-at ?at)
     (not (available ?at))
     (openable ?arm ?at))
  :effect
  (and (available ?at) (open ?at) (increase (total-cost) (door-cost)))
  :duration
  nil)
 (:action
  "close-door"
  :parameters
  (?arm - arm ?at - spot)
  :precondition
  (and
     (forall
      (?obj - item)
      (and (not (onhand ?obj ?arm)) (not (onhand ?obj arms))))
     (robot-at ?at)
     (openable ?arm ?at)
     (open ?at)
     (available ?at))
  :effect
  (and
     (not (available ?at))
     (not (open ?at))
     (increase (total-cost) (door-cost)))
  :duration
  nil)
 (:action
  "clean-room"
  :parameters
  (?arm - arm ?at - spot)
  :precondition
  (and (onhand cleaner ?arm) (robot-at ?at))
  :effect
  (and (clean) (not (robot-at ?at)) (robot-at somewhere))
  :duration
  nil)
 (:action
  "find-person"
  :parameters
  (?who - person ?at - spot)
  :precondition
  (and (robot-at ?at) (see ?who ?at) (available ?at))
  :effect
  (and (sure-see ?who ?at) (not (see ?who ?at)))
  :duration
  nil)
 (:action
  "deliver"
  :parameters
  (?obj - item ?arm - arm ?who - person ?at - spot)
  :precondition
  (and
     (onhand ?obj ?arm)
     (robot-at ?at)
     (sure-see ?who ?at)
     (available ?at))
  :effect
  (and (not (onhand ?obj ?arm)) (delivered ?obj ?who))
  :duration
  nil)
 (:action
  "ask"
  :parameters
  (?who - person ?at - spot)
  :precondition
  (and (robot-at ?at) (sure-see ?who ?at) (available ?at))
  :effect
  (and (asked ?who))
  :duration
  nil)
 (:action
  "find-object_f"
  :parameters
  (?obj - item ?at - spot)
  :precondition
  (and (robot-at ?at) (on ?obj ?at) (available ?at))
  :effect
  (and (not (on ?obj ?at)))
  :duration
  nil)
 (:action
  "pick_f"
  :parameters
  (?obj - item ?arm - arm ?at - spot)
  :precondition
  (and
     (forall
      (?other - item)
      (and (not (onhand ?other ?arm)) (not (onhand ?other arms))))
     (robot-at ?at)
     (sure-on ?obj ?at)
     (available ?at))
  :effect
  (and
     (on ?obj ?at)
     (not (sure-on ?obj ?at))
     (increase (total-cost) (manip-cost ?obj ?arm)))
  :duration
  nil)
 (:action
  "find-person_f"
  :parameters
  (?who - person ?at - spot)
  :precondition
  (and (robot-at ?at) (see ?who ?at) (available ?at))
  :effect
  (and (not (see ?who ?at)))
  :duration
  nil))
