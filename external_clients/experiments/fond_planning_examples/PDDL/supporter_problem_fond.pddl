(define (problem supporter-fond)
(:domain robocup-fond)
(:objects 
  supporter-robot
  striker-robot
    - movable 
  
  striker-current-position 
  supporter-current-position 
  supporter-receive-pass-position
    - location
)

(:init 
  (is-at supporter-robot supporter-current-position) 
  (is-at striker-robot striker-current-position) 
)

(:goal 
  (or 
    (is-at supporter-robot supporter-receive-pass-position)
  )
)
)