from enum import Enum

#lower value => higher priority in state checking.
class State(Enum):
    COLLIDING = 1
    HUMAN_CONTROLLING = 2
    ESCAPE_SYMMETRIC = 3
    AVOID_ASYMMETRIC = 4
    TURN_RANDOMLY = 5
    DRIVE_FORWARD = 6