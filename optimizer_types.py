# Copyright (c) 2026 Justin Kleiber

from dataclasses import dataclass

@dataclass 
class Constraint:
    min: float
    max: float


@dataclass
class ProjectileMotionConstraints:
    distance: Constraint
    flywheel_rps: Constraint
    launch_angle: Constraint


@dataclass
class TargetInfo:
    delta_height: float
    arrival_angle: float
    distance: float
    height: float