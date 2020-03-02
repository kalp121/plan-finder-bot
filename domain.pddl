(define (domain bookWorld)

    (:requirements
        :equality
        :typing
        :strips
    )

    (:types
        robot
        book
        bin
        location
        subject
        size
    )


    (:predicates
        (Book_At ?a - book ?b - location)
        (Bin_At ?a - bin ?b - location)
        (Book_Subject ?a - book ?b - subject)
        (Book_Size ?a - book ?b - size)
        (Bin_Subject ?a - bin ?b - subject)
        (Bin_Size ?a - bin ?b - size)
        (Robot_At ?a - robot ?b - location)
        (Empty_Basket ?a - robot)
        (In_Basket ?b - book)
    )


    (:action pick
        :parameters (?book - book
                     ?location - location
                     ?robot - robot)
        :precondition (
            and (Book_At ?book ?location)(Robot_At ?robot ?location)(Empty_Basket ?robot)
        )
        :effect (
            and (not (Book_At ?book ?location))(In_Basket ?book)(not (Empty_Basket ?robot))
        )
    )

    (:action place
        :parameters (?book - book
                     ?subject - subject
                     ?bin - bin
                     ?size - size
                     ?location - location
                     ?robot - robot)
        :precondition (
            and (not (Empty_Basket ?robot))(Robot_At ?robot ?location)(Bin_At ?bin ?location)(In_Basket ?book)(Book_Subject ?book ?subject)(Bin_Subject ?bin ?subject)(Bin_Size ?bin ?size)(Book_Size ?book ?size)
        )
        :effect (
            and (Empty_Basket ?robot)(not (In_Basket ?book))(Book_At ?book ?location)
        )
    )

    (:action move
        :parameters (?robot - robot
                     ?location-from - location
                     ?location-to - location)
        :precondition (
            Robot_At ?robot ?location-from
        )
        :effect (
            and (not (Robot_At ?robot ?location-from))(Robot_At ?robot ?location-to)
        )
    )
)