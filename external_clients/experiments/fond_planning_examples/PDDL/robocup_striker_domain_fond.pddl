;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Non-Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain robocup-fond)
  (:requirements :strips :typing :existential-preconditions)
  (:types movable location)
  (:predicates 
		   (is-opponent-blocking-goal)
           (goal-scored)
		   (ball-passed)
		   (striker-has-ball)
		   (striker-can-kick)
	       )

  (:action move-to-ball
	     :precondition 
		 (and 
			(not (striker-has-ball))
		 )
	     :effect
	     (oneof 
		 	(and (is-opponent-blocking-goal) (striker-has-ball))
		 	(and (not (is-opponent-blocking-goal)) (striker-has-ball))
		 )
  )

  (:action move-to-kicking-position
	     :precondition 
		 (and 
		 	(not (is-opponent-blocking-goal))
			(not (striker-can-kick))
			(striker-has-ball)
		 )
	     :effect
	     (oneof 
		 	(and (is-opponent-blocking-goal) (striker-has-ball) (striker-can-kick))
		 	(and (not (is-opponent-blocking-goal)) (striker-has-ball) (not(striker-can-kick)))
		 )
  )

  (:action kick-to-goal
	     :precondition 
		 	(and 
			 	(not (is-opponent-blocking-goal)) 
				(striker-can-kick)
			)
	     
		 :effect
	     	(and 
			 (goal-scored)
			)
  )

  (:action pass-ball-to-supporter
	     :precondition 
		 	(and 
		 		(is-opponent-blocking-goal) 
				(not(striker-can-kick))
			)
	     :effect
	     (and 
			(ball-passed)
		 )
  )
  
)
