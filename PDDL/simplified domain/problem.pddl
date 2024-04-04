(define (problem robot-problem)

	(:domain robot-domain)

	(:objects R P L1 L2)

	(:init (robot R)(person P)
	       (level L1)(level L2))
	
	(:goal (and(not(wait_status R))(person_detected P)(person_ready P)(session_on R P)(all_levels_completed P)))
	
)