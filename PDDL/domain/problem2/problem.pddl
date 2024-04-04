(define (problem robot-problem)

	(:domain robot-domain)

	(:objects R P HELLO L1 L2 IM1 IM2 IM3 IM4 )

	(:init (robot R)(person P)(level L1)(level L2)
	       (image IM1)(image IM2)(image IM3)(image IM4)
	       (different IM1 IM2)(different IM1 IM3)(different IM1 IM4)
	       (different IM2 IM3)(different IM2 IM4)(different IM3 IM4)
	       (different L1 L2)
	       (first_level L1)(prec L1 L2)
	       (level_image L1 IM1)(level_image L1 IM2)
	       (level_image L2 IM3)(level_image L2 IM4)
	)
	
	(:goal (and(not(wait_status R))(face_detected P)(keyword_detected HELLO)
	           (person_ready P)(all_levels_completed P) ) )
	
)