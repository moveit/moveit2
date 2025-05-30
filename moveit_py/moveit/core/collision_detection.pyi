from _typeshed import Incomplete

class AllowedCollisionMatrix:
    def __init__(self, *args, **kwargs) -> None: ...
    def clear(self, *args, **kwargs): ...
    def get_entry(self, *args, **kwargs): ...
    def remove_entry(self, *args, **kwargs): ...
    def set_entry(self, *args, **kwargs): ...

class CollisionRequest:
    contacts: Incomplete
    cost: Incomplete
    distance: Incomplete
    joint_model_group_name: Incomplete
    max_contacts: Incomplete
    max_contacts_per_pair: Incomplete
    max_cost_sources: Incomplete
    verbose: Incomplete
    def __init__(self, *args, **kwargs) -> None: ...

class CollisionResult:
    collision: Incomplete
    contact_count: Incomplete
    contacts: Incomplete
    cost_sources: Incomplete
    distance: Incomplete
    def __init__(self, *args, **kwargs) -> None: ...
