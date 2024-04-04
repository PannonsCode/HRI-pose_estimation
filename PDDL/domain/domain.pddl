(define (domain robot-domain)

(:requirements :strips)

(:predicates (robot ?r)(person ?p)(level ?l)(image ?im)(level_image ?l ?im)(shown_image ?im)
             (wait_status ?r)(face_detected ?p)(keyword_detected ?w)(known_person ?p)(data_inserted ?p)
             (person_ready ?p)(reg_completed ?p)(session_on ?r ?p)
             (level_started ?l)(level_completed ?l)(prec ?l1 ?l2)(first_level ?l)
             (mark_assigned ?l)(all_levels_completed ?p)(different ?l1 ?l2)(talk ?r ?p)
)

(:action wait
    :parameters (?p)
    :precondition (and(person ?p)(not(talk R P)))
    :effect (wait_status R)
)

(:action detect_face
    :parameters (?p)
    :precondition (and (person ?p)(not(face_detected ?p))(not(talk R P))
                  (or (wait_status R)(keyword_detected HELLO)))
    :effect (and(face_detected ?p)(not(wait_status R)))
)

(:action listen
    :parameters (?p)
    :precondition (and(person ?p)(not(keyword_detected HELLO))(not(talk R P))
                  (or (wait_status R)(face_detected ?p)))
    :effect (keyword_detected HELLO)
)

(:action insert_data
    :parameters (?p)
    :precondition (and (person ?p)(face_detected ?p)(keyword_detected HELLO)
                  (not (data_inserted ?p))(talk R P))
    :effect (and(data_inserted ?p)(not(talk R ?p)))
)

(:action complete_registration
    :parameters (?p)
    :precondition (and (person ?p)(data_inserted ?p)(not (known_person ?p))(not (reg_completed ?p))(not(talk R P)))
    :effect (and (reg_completed ?p))
)

(:action store_data
    :parameters (?p)
    :precondition (and (person ?p)(data_inserted ?p)(reg_completed ?p)(not (person_ready ?p))(not(talk R P)))
    :effect (person_ready ?p)
)

(:action recover_data
    :parameters (?p)
    :precondition (and (person ?p)(known_person ?p)(data_inserted ?p)(not (reg_completed ?p))(not (person_ready ?p))(not(talk R P)))
    :effect (person_ready ?p)
)

(:action start_session
    :parameters (?p ?l)
    :precondition (and (person ?p)(person_ready ?p)(not (session_on R ?p))
                  (level ?l)(first_level ?l)(not (level_started ?l))(talk R ?p))
    :effect (and (session_on R ?p)(level_started ?l)(not(talk R P)))
)

(:action show_image
    :parameters (?l ?im)
    :precondition (and (level ?l)(level_started ?l)(image ?im)(level_image ?l ?im)(not (shown_image ?im)))
    :effect (shown_image ?im)
)

(:action assign_mark
    :parameters (?im)
    :precondition (and (image ?im)(shown_image ?im)(not (mark_assigned ?im)))
    :effect (and (mark_assigned ?im)(not(talk R P)))
)

(:action check_level
    :parameters (?l ?im1 ?im2)
    :precondition (and (level ?l)(level_image ?l ?im1)(level_image ?l ?im2)(different ?im1 ?im2)
                  (mark_assigned ?im1)(mark_assigned ?im2)(not(level_completed ?l)))
    :effect (level_completed ?l)
)

(:action next_level
    :parameters(?l1 ?l2)
    :precondition (and (level ?l1)(level ?l2)(prec ?l1 ?l2)(level_completed ?l1)
                  (not (level_started ?l2))(talk R P))
    :effect (and(level_started ?l2)(not(talk R P)))
)

(:action end_session
    :parameters (?p)
    :precondition (and (person ?p)(level_completed L1)(level_completed L2)(talk R ?p))
    :effect (all_levels_completed ?p)
)

(:action interact
    :parameters (?p)
    :precondition (person ?p)
    :effect (talk R ?p)
)

)



















