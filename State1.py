#!/usr/bin/env pybricks-micropython
from enum import Enum

class State(Enum):
    IDLE = 0
    FOLLOW_LINE = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
