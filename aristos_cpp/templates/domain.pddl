(define (domain coverage)
    (:requirements :typing :negative-preconditions :disjunctive-preconditions)
    (:types cell direction)
    (:predicates
        (at ?c - cell)
        (visited ?c - cell)
        (towards ?d - direction)
        (north ?c1 - cell ?c2 - cell ?d - direction)
        (east ?c1 - cell ?c2 - cell ?d - direction)
        (south ?c1 - cell ?c2 - cell ?d - direction)
        (west ?c1 - cell ?c2 - cell ?d - direction)
        (perpendicular ?d1 - direction ?d2 - direction)
        (unsafe ?c - cell)
    )
    (:action move
        :parameters (?from - cell ?to - cell ?d - direction)
        :precondition (and 
            (at ?from)
            (towards ?d)
            (not (visited ?to))
            (not (unsafe ?from))
            (or
                (north ?from ?to ?d)
                (south ?from ?to ?d)
                (west ?from ?to ?d)
                (east ?from ?to ?d)
            )    
        )
        :effect (and 
            (visited ?to) 
            (not (at ?from)) 
            (at ?to)
        )
    )

    (:action move-unsafe-vertical
        :parameters (?from - cell ?to - cell ?d - direction)
        :precondition (and 
            (at ?from)
            (towards ?d)
            (not (visited ?to))
            (unsafe ?from)
            (or
                (north ?from ?to ?d)
                (south ?from ?to ?d)
                
            )    
           
        )
        :effect (and 
            (visited ?to)
            (not (at ?from))
            (at ?to)
            
        )
    )
    (:action revisit
        :parameters (?from - cell ?to - cell ?d - direction)
        :precondition (and 
            (at ?from)
            (towards ?d)
            (visited ?to)
            (or
                (north ?from ?to ?d)
                (south ?from ?to ?d)
                (west ?from ?to ?d)
                (east ?from ?to ?d)
            )    
        )
        :effect (and 
            (not (at ?from)) 
            (at ?to)
        )
    )
    (:action revisit-unsafe-vertical
        :parameters (?from - cell ?to - cell ?d - direction)
        :precondition (and 
            (at ?from)
            (towards ?d)
            (visited ?to)
            (unsafe ?from)
            (or
                (north ?from ?to ?d)
                (south ?from ?to ?d)
            )    
           
        )
        :effect (and 
            (not (at ?from))
            (at ?to)
            
        )
    )
    (:action turn
        :parameters (?d1 - direction ?d2 - direction)
        :precondition (and (towards ?d1) (perpendicular ?d1 ?d2))
        :effect (and 
            (towards ?d2) 
            (not (towards ?d1))
        )
    )
)