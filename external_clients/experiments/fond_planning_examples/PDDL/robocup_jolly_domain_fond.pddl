;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Non-Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain jolly-robocup-fond)
  (:requirements :strips)
  (:predicates 
    (fluent-is-striker-obstacle-blocking-goal)
    (fluent-is-jolly-in-position)
    (fluent-is-jolly-aligned-to-striker)
    (jolly-in-position)
    (jolly-aligned-to-striker)
    (jolly-available)
  )

  (:action check-obstacle-position
	     :precondition 
        (jolly-available)
       :effect
	     	(oneof
          (fluent-is-striker-obstacle-blocking-goal)
          (when (not (fluent-is-striker-obstacle-blocking-goal)) (jolly-in-position))
        )
  )
  (:action move-to-receiving-position
	     :precondition 
       (and
          (fluent-is-striker-obstacle-blocking-goal)
          (not (jolly-in-position))
        )
       :effect
        (jolly-in-position)
  )

  (:action turn-to-striker
	     :precondition 
       (and
          (jolly-in-position)
          (or
            (not(jolly-aligned-to-striker))
            (not(fluent-is-striker-obstacle-blocking-goal))
          )
       )
       :effect
       (and 
          (jolly-aligned-to-striker)
       )
  ) 

  (:action check-jolly-position
	     :precondition 
       (not (jolly-position-ok))
       :effect
        (and
          (when (fluent-is-jolly-in-position) (jolly-position-ok))
          (when (jolly-in-position) (jolly-position-ok))
        )
  )

  (:action check-jolly-rotation
	     :precondition 
       (not (jolly-rotation-ok))
       :effect
        (and
          (when (fluent-is-jolly-aligned-to-striker) (jolly-rotation-ok))
          (when (jolly-aligned-to-striker) (jolly-rotation-ok))
        )
  )

  (:action check-jolly-ready
	     :precondition
       (and 
          (jolly-position-ok)
          (jolly-rotation-ok)
        )
       :effect
        (jolly-ready)
  )
  
)
