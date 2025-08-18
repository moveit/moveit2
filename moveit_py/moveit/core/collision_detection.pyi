from typing import Any

class AllowedCollisionMatrix:
    def __init__(self, *args, **kwargs) -> None: ...
    def clear(self, *args, **kwargs) -> Any: ...
    def get_entry(self, *args, **kwargs) -> Any: ...
    def remove_entry(self, *args, **kwargs) -> Any: ...
    def set_entry(self, *args, **kwargs) -> Any: ...

class CollisionRequest:
    contacts: Any
    cost: Any
    distance: Any
    joint_model_group_name: Any
    max_contacts: Any
    max_contacts_per_pair: Any
    max_cost_sources: Any
    verbose: Any
    def __init__(self, *args, **kwargs) -> None: ...

class CollisionResult:
    collision: Any
    contact_count: Any
    contacts: Any
    cost_sources: Any
    distance: Any
    def __init__(self, *args, **kwargs) -> None: ...
