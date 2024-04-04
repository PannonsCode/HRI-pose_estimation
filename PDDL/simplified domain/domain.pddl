(define (domain robot-domain)

(:requirements :strips)

(:predicates (robot ?r)(person ?p)(level ?level)(level_completed ?l)
             (wait_status ?r)(person_detected ?p)(person_ready ?p)
             (session_on ?r ?p)(all_levels_completed ?p)
)

(:action wait
    :parameters (?p)
    :precondition (and(person ?p)(not(person_detected ?p)))
    :effect (wait_status R)
)

(:action detect_person
    :parameters (?p)
    :precondition (and (person ?p)(not(person_detected ?p))(wait_status R))
    :effect (and(person_detected ?p)(not(wait_status R)))
)

(:action identify_person
    :parameters (?p)
    :precondition (and (person ?p)(person_detected ?p)(not(person_ready ?p)))
    :effect (and (person_ready ?p))
)

(:action start_session
    :parameters (?p)
    :precondition (and (person ?p)(person_ready ?p)(not (session_on R ?p)))
    :effect (session_on R ?p)
)

(:action run_level
    :parameters (?l)
    :precondition (and (level ?l)(not (level_completed ?l)))
    :effect (level_completed ?l)
)

(:action end_session
    :parameters (?p)
    :precondition (and (person ?p)(session_on R ?p)(not(all_levels_completed ?p))(level_completed L1)(level_completed L2))
    :effect (and (all_levels_completed ?p))
)

)



















