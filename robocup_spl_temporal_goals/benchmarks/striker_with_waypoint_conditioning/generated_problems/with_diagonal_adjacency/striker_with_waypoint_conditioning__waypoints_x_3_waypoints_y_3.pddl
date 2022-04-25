(define (problem simplestriker)
(:domain robocupdeterministic)
(:objects robot ball - movable 
    wpc0r0 wpc0r1 wpc0r2 wpc1r0 wpc1r1 wpc1r2 wpc2r0 wpc2r1 wpc2r2 - location
    )
(:init 
    (isrobot robot) (isball ball) 
(adjacent wpc0r1 wpc0r0)
(adjacent wpc0r1 wpc1r0)
(adjacent wpc0r2 wpc0r1)
(adjacent wpc0r2 wpc1r1)
(adjacent wpc1r0 wpc0r0)
(adjacent wpc1r1 wpc0r1)
(adjacent wpc1r1 wpc1r0)
(adjacent wpc1r1 wpc0r0)
(adjacent wpc1r1 wpc2r0)
(adjacent wpc1r2 wpc0r2)
(adjacent wpc1r2 wpc1r1)
(adjacent wpc1r2 wpc0r1)
(adjacent wpc1r2 wpc2r1)
(adjacent wpc2r0 wpc1r0)
(adjacent wpc2r1 wpc1r1)
(adjacent wpc2r1 wpc2r0)
(adjacent wpc2r1 wpc1r0)
(adjacent wpc2r2 wpc1r2)
(adjacent wpc2r2 wpc2r1)
(adjacent wpc2r2 wpc1r1)

(isat robot wpc0r1)
 (isat ball wpc1r1)

)
(:goal 
    (isat ball wpc2r1)

)
)