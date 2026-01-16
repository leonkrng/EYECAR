import time
from aruco_navigation.movement_base import MovementBase
from aruco_navigation.movement_enum import MovementEnum


class MovementPickAndPlace(MovementBase):
    def __init__(self):
        super().__init__()


    def marker_1_movement(self):
        
        match self.current_sub_step:
            case 0:
                # Gripper down
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if (time.monotonic_ns() - self.sub_step_start_time) > self.LOWER_GRIPPER_DURATION:
                    self.current_sub_step = 1
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_DOWN

            case 1:
                # Gripper open
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.OPEN_GRIPPER_DURATION:
                    self.current_sub_step = 2
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_OPEN
        
            case 2: 
                # Get close to the workstation 
                if self.workstation_distance <= self.GRIP_DISTANCE:
                    self.current_sub_step = 3
                    return MovementEnum.STOP
                else:
                    return MovementEnum.FORWARD
            case 3:
                # Close Gripper 
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.CLOSE_GRIPPER_DURATION:
                    self.current_sub_step = 4
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_CLOSE

            case 4:
                # Gripper Up
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.LIFT_GRIPPER_DURATION:
                    self.current_sub_step = 5
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_UP

            case 5:
                # Backup from workstation
                if self.workstation_distance > (self.GRIP_DISTANCE + 0.1):
                    self.current_sub_step = 6
                    return MovementEnum.STOP
                else:
                    return MovementEnum.BACKWARD

            case 6:
                # Rotate 180°
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.ROTATE_180_DURATION:
                    self.current_sub_step = self.SUB_ROUTINE_DONE 
                    return MovementEnum.STOP
                else:
                    return MovementEnum.TURN_RIGHT

            case _: 
                return MovementEnum.STOP

    def marker_2_movement(self):

        match self.current_sub_step:
            case 0:
                # Get close to workstation
                if self.workstation_distance <= self.GRIP_DISTANCE:
                    self.current_sub_step = 1
                    return MovementEnum.STOP
                else:
                    return MovementEnum.FORWARD

            case 1:
                # Lower Gripper
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.LOWER_GRIPPER_DURATION:
                    self.current_sub_step = 2
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_DOWN

            case 2:
                # Open Gripper
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.OPEN_GRIPPER_DURATION:
                    self.current_sub_step = 3
                    self.sub_step_start_time = None
                    return MovementEnum.STOP
                else:
                    return MovementEnum.GRIPPER_OPEN

            case 3:
                # Backup from workstation
                if self.workstation_distance > (self.GRIP_DISTANCE + 0.1):
                    self.current_sub_step = 4 
                    return MovementEnum.STOP
                else:
                    return MovementEnum.BACKWARD

            case 4:
                # Rotate 180°
                if self.sub_step_start_time is None:
                    self.sub_step_start_time = time.monotonic_ns()

                if(time.monotonic_ns() - self.sub_step_start_time) > self.ROTATE_180_DURATION:
                    self.current_sub_step = self.SUB_ROUTINE_DONE 
                    return MovementEnum.STOP
                else:
                    return MovementEnum.TURN_RIGHT
            
            case _:
                return MovementEnum.STOP

    def marker_3_movement(self):
        return MovementEnum.STOP

    def marker_4_movement(self):
        return MovementEnum.STOP
