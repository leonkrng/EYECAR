from aruco_navigation.movement_enum import MovementEnum

# Aligns the EYECAR to the marker. 
def align_to_marker(frame, current_marker):

    is_aligned = False

    if current_marker is None:
        return MovementEnum.STOP, is_aligned

    height, width = frame.shape[:2]

    # The span between 40% and 60% off the frame is considered to be the middle
    border_left = int(width * 0.4)
    border_right = int(width * 0.6)

    # A <20% difference between the lengt of the sides is considered straight
    side_diff = current_marker.side_TL_BL / current_marker.side_TR_BR

    # If the marker the diagonales are 50% of the widht the marker is considered close enough
    max_size = 0.3 * width 

    if current_marker.center_x < border_left:
        # Marker too far to the left
        return MovementEnum.LEFT, is_aligned

    if current_marker.center_x > border_right:
        # Marker too far to the right
        return MovementEnum.RIGHT, is_aligned

    if side_diff > 1.1 :
        # Marker is seen from the left side
        return MovementEnum.TURN_LEFT, is_aligned

    if side_diff < 0.9:
        # Marker is seen from the right side
        return MovementEnum.TURN_RIGHT, is_aligned


    if current_marker.diag_TL_BR < max_size or current_marker.diag_TR_BL < max_size:
        # Marker is too far away
        return MovementEnum.FORWARD, is_aligned

    
    if current_marker.diag_TL_BR >= max_size and current_marker.diag_TR_BL >= max_size:
        # Marker is close enough and aligend
        is_aligned = True
        print("The EYECAR is now aligned to the marker.")
        return MovementEnum.STOP, is_aligned

    # Fallback 
    return MovementEnum.STOP, is_aligned
