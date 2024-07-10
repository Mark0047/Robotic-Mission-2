(define (domain example)
	(:requirements :typing)
	(:types
		localisable location - object
		room - location
		robot - localisable
	)
	(:predicates
		(in ?obj - localisable ?loc - location)
		(cangonorth ?from - location ?to - location)
		(cangoeast ?from - location ?to - location)
		(cangosouth ?from - location ?to - location)
		(cangowest ?from - location ?to - location)
	)
	(:action movenorth
		:parameters (
			?bot - object
			?from - location
			?to - location
		)
		:precondition (
		and
			(in ?bot ?from)
			(cangonorth ?from ?to)
		)
		:effect (
		and
			(in ?bot ?to)
			(not (in ?bot ?from))
		)
	)
	(:action moveeast
		:parameters (
			?bot - object
			?from - location
			?to - location
		)
		:precondition (
		and
			(in ?bot ?from)
			(cangoeast ?from ?to)
		)
		:effect (
		and
			(in ?bot ?to)
			(not (in ?bot ?from))
		)
	)
	(:action movesouth
		:parameters (
			?bot - object
			?from - location
			?to - location
		)
		:precondition (
		and
			(in ?bot ?from)
			(cangosouth ?from ?to)
		)
		:effect (
		and
			(in ?bot ?to)
			(not (in ?bot ?from))
		)
	)
	(:action movewest
		:parameters (
			?bot - object
			?from - location
			?to - location
		)
		:precondition (
		and
			(in ?bot ?from)
			(cangowest ?from ?to)
		)
		:effect (
		and
			(in ?bot ?to)
			(not (in ?bot ?from))
		)
	)
)
