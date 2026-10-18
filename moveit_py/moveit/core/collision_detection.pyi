import enum
from typing import Any

import numpy as np


class AllowedCollisionMatrix:
    def __init__(self, *args, **kwargs) -> None: ...
    def clear(self, *args, **kwargs) -> Any: ...
    def get_entry(self, *args, **kwargs) -> Any: ...
    def remove_entry(self, *args, **kwargs) -> Any: ...
    def set_entry(self, *args, **kwargs) -> Any: ...


class BodyType(enum.Enum):
    ROBOT_LINK: int
    ROBOT_ATTACHED: int
    WORLD_OBJECT: int


class Contact:
    body_name_1: str
    body_name_2: str
    body_type_1: BodyType
    body_type_2: BodyType
    depth: float
    pos: np.ndarray
    normal: np.ndarray
    percent_interpolation: float
    nearest_points: tuple[np.ndarray, np.ndarray]
    def __init__(self, *args, **kwargs) -> None: ...


class CostSource:
    aabb_min: list[float]
    aabb_max: list[float]
    cost: float
    volume: float
    def __init__(self, *args, **kwargs) -> None: ...
    def get_volume(self) -> float: ...


class CollisionRequest:
    contacts: bool
    cost: bool
    distance: bool
    joint_model_group_name: str
    max_contacts: int
    max_contacts_per_pair: int
    max_cost_sources: int
    verbose: bool
    def __init__(self, *args, **kwargs) -> None: ...


class CollisionResult:
    collision: bool
    contact_count: int
    contacts: dict[tuple[str, str], list[Contact]]
    cost_sources: list[CostSource]
    distance: float
    def __init__(self, *args, **kwargs) -> None: ...
    def clear(self) -> None: ...
