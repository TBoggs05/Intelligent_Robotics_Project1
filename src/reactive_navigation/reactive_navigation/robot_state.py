"""
State enumeration for reactive navigation subsumption architecture.

Defines the behavioral states of the robot ordered strictly by their
subsumption priority (1 = highest priority, 6 = lowest priority).
"""

from enum import Enum


class State(Enum):
    """
    Behavior states ordered by priority (lower numeric value = higher priority).

    Subsumption Priority Hierarchy:
        1. COLLIDING: Physical bumper contact detected; halt immediately.
        2. HUMAN_CONTROLLING: Operator teleoperation via keyboard override.
        3. ESCAPE_SYMMETRIC: Head-on obstacle within 1 ft; execute 180° ± 30° turn.
        4. AVOID_ASYMMETRIC: Side obstacle within 1 ft; reflexive steering away.
        5. TURN_RANDOMLY: Exploration turn (±15°) after 1 ft forward motion.
        6. DRIVE_FORWARD: Default basal behavior; drive forward at 0.25 m/s.
    """

    COLLIDING = 1
    HUMAN_CONTROLLING = 2
    ESCAPE_SYMMETRIC = 3
    AVOID_ASYMMETRIC = 4
    TURN_RANDOMLY = 5
    DRIVE_FORWARD = 6
