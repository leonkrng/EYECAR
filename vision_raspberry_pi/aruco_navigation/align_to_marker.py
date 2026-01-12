from aruco_navigation.movement_enum import MovementEnum

    # Aligns the EYECAR to the marker. 
    def align_to_marker(frame, current_marker):

        aligned = False

        height, width = frame.shape[:2]

        # The span between 40% and 60% off the frame is considered to be the middle
        border_left = int(width * 0.4)
        border_right = int(width * 0.6)

        # A <10% difference between the lengt of the sides is considered straight
        side_diff = current_marker.side_TL_BL / current_marker.side_TR_BR

        # If the marker the diagonales are 40% of the widht the marker is considered close enough
        max_size = 0.4 * width 

        if current_marker.center_x < border_left:
            # Marker too far to the left
            return MovementEnum.LEFT, aligned

        if current_marker.center_x > border_right:
            # Marker too far to the right
            return MovementEnum.RIGHT, aligned

        if side_diff > 1.1 :
            # Marker is seen from the left side
            return MovementEnum.TURN_LEFT, aligned

        if side_diff < 0.9:
            # Marker is seen from the right side
            return MovementEnum.TURN_RIGHT, aligned

        if current_marker.center_x > border_left and current_marker.center_x < border_right:
            # Marker is in the middle but too far away
            return MovementEnum.FORWARD, aligned 

        if current_marker.diag_TL_BR > max_size and current_marker.diag_TR_BL > max_size:
            # Marker is close enough and aligend
            aligned = True
            return MovementEnum.STOP, aligned
        
        # Fallback 
        return MovementEnum.STOP, aligned
