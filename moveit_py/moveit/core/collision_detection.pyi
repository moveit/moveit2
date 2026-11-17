from __future__ import annotations

from typing import Any, ClassVar

import numpy as np


class AllowedCollisionMatrix:
    def __init__(self, *args, **kwargs) -> None: ...
    def clear(self, *args, **kwargs) -> Any: ...
    def get_entry(self, *args, **kwargs) -> Any: ...
    def remove_entry(self, *args, **kwargs) -> Any: ...
    def set_entry(self, *args, **kwargs) -> Any: ...


class BodyType:
    ROBOT_LINK: ClassVar[BodyType]
    ROBOT_ATTACHED: ClassVar[BodyType]
    WORLD_OBJECT: ClassVar[BodyType]
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __index__(self) -> int: ...
    def __eq__(self, other: object) -> bool: ...
    def __ne__(self, other: object) -> bool: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...


ROBOT_LINK: BodyType
ROBOT_ATTACHED: BodyType
WORLD_OBJECT: BodyType


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
