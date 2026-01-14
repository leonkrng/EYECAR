from abc import ABC, abstractmethod

class MovementBase:
    def __init__(self):
        
        self.current_sub_step = 0
        self.SUB_ROUTINE_DONE = 999
        self.sub_step_start_time = None
        self.workstation_distance = 0

        # Constans for the duration that the movement needs
        self.LOWER_GRIPPER_DURATION = 4 * 1000000000
        self.LIFT_GRIPPER_DURATION = 4 * 1000000000
        self.CLOSE_GRIPPER_DURATION = 10 * 1000000000
        self.OPEN_GRIPPER_DURATION = 10 * 1000000000
        self.RORARE_180_DURATION = 2 * 1000000000
 
        # Distance between the lidar and the workstation for gripping the object (in m)
        self.GRIP_DISTANCE = 0.2

    def movement_routine(self, marker_id):

        match marker_id:
            case 1:
                return self.marker_1_movement()
            case 2:
                return self.marker_2_movement()
            case 3:
                return self.marker_3_movement()
            case 4:
                return self.marker_4_movement()

    
    @abstractmethod
    def marker_1_movement():
        pass

    @abstractmethod
    def marker_2_movement():
        pass

    @abstractmethod
    def marker_3_movement():
        pass

    @abstractmethod
    def marker_4_movement():
        pass


