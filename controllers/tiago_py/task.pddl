(define (problem go-to) (:domain example)
	(:objects
		bot - robot
		redroom greenroom ballsroom ducksroom - room
	)
	(:init
		(in bot ducksroom)
		(cangowest greenroom redroom)
		(cangowest ballsroom ducksroom)

		(cangoeast redroom greenroom)
		(cangoeast ducksroom ballsroom)

		(cangonorth ducksroom redroom)
		(cangonorth ballsroom greenroom)

		(cangosouth greenroom ballsroom)
		(cangosouth redroom ducksroom)

	)
	(:goal
		(in bot redroom)
	)
    )