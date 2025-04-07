from typing import List, Tuple, Dict
import collision_detection

class AllowedCollisionMatrix:
    def __init__(self, names: List[str], default_entry: bool = False) -> None:
        """
        Initialize the allowed collision matrix using a list of names of collision objects.

        Args:
            names: A list of names of the objects in the collision world.
            default_entry: If false, indicates that collisions between all elements must be checked for and no collisions will be ignored.
        """
        ...
    def get_entry(
        self, name1: str, name2: str
    ) -> Tuple[bool, collision_detection.AllowedCollision.Type]:
        """
        Get the allowed collision entry for a pair of objects.

        Args:
            name1: The name of the first object.
            name2: The name of the second object.

        Returns:
            A tuple containing a boolean indicating if the collision is allowed, and the type of allowed collision.
        """
        ...
    def set_entry(self, name1: str, name2: str, allowed: bool) -> None:
        """
        Set the allowed collision state between two objects.

        Args:
            name1: The name of the first object.
            name2: The name of the second object.
            allowed: If true, indicates that the collision between the two objects is allowed. If false, it is not allowed.
        """
        ...
    def remove_entry(self, name1: str, name2: str) -> None:
        """
        Remove an entry corresponding to a pair of elements from the collision matrix.
        Nothing happens if the pair does not exist in the matrix.

        Args:
            name1: The name of the first object.
            name2: The name of the second object.
        """
        ...
    def clear(self) -> None:
        """
        Clear the allowed collision matrix.
        """
        ...

class CollisionRequest:
    def __init__(self) -> None:
        """
        Representation of a collision checking request.
        """
        ...
    joint_model_group_name: str
    """ The group name to check collisions for (optional if empty, assume the complete robot) """

    distance: bool
    """ If true, compute proximity distance """

    cost: bool
    """ If true, compute collision cost """

    contacts: bool
    """ If true, compute contacts """

    max_contacts: int
    """Maximum number of contacts to compute """

    max_contacts_per_pair: int
    """ Maximum number of contacts to compute per pair of bodies """

    max_cost_sources: int
    """ The number of top cost sources to return when costs are computed """

    verbose: bool
    """ Flag indicating whether collision information should be reported """

class CollisionResult:
    def __init__(self) -> None:
        """
        Representation of a collision checking result.
        """
        ...
    collision: bool
    """ True if a collision was found, False otherwise """

    distance: float
    """ The closest distance between two bodies """

    contact_count: int
    """ The number of contacts returned """

    contacts: Dict[str, any]
    """ A dictionary of pairs of ids of bodies in contact, plus information about the contacts themselves """

    cost_sources: Dict[str, any]
    """ Individual cost sources from computed costs """
