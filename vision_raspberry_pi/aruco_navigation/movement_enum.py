from enum import Enum
from pickle import STOP

class MovementEnum(Enum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    FORWARD_LEFT = 5
    FORWARD_RIGHT = 6
    BACKWARD_LEFT = 7
    BACKWARD_RIGHT = 8
    TURN_LEFT = 9
    TURN_RIGHT = 10
    FORWARD_TURN_LEFT = 11
    FORWARD_TURN_RIGHT = 12
    BACKWARD_TURN_LEFT = 13
    BACKWARD_TURN_RIGHT = 14

    GRIPPER_DOWN = 15 # Gripping-Position
    GRIPPER_UP = 16 # Transport-Position
    GRIPPER_OPEN = 17
    GRIPPER_CLOSE = 18

    LED_ON = 19
    LED_OFF = 20
