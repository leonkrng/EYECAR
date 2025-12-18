from enum import Enum
from pickle import STOP

class MovementEnum(Enum):
    STOP = 0
    NO_MARKER = 1
    ALIGN_RIGHT = 2
    FORWARD = 3
    ALIGN_LEFT = 4
    SEARCH_NEXT = 5
    TOO_CLOSE = 6

