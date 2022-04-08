;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Non-Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain robocup-fond)
  (:requirements :strips :typing :existential-preconditions)
  (:types movable location)
  (:predicates 
           (is-robot ?mov - movable)
           (is-at ?mov - movable ?loc - location)
	       )

  (:action move-robot
	     :parameters (?rob - movable ?from - location ?to - location)
	     :precondition (and 
		 	(is-robot ?rob) 
			(is-at ?rob ?from)
			(not (is-at ?rob ?to))
        )
	     :effect
        (and (not (is-at ?rob ?from)) (is-at ?rob ?to))
  )
  
)
