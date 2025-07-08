import cv2 
import cv2.aruco as aruco

from aruco_navigation import marker_visualization
from aruco_navigation import calc_navigation
from aruco_navigation import MovementEnum

def read_marker(frame, prev_aruco_navigation_active, navigation_list, max_marker_size, camera_resolution, actual_ID):

    # Turn frame into grayscale
    grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Get ArUco-Dictionary
    key = getattr(aruco, f"DICT_5X5_50")
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()

    # Read ArUco
    (corners, ids, rejected) = aruco.detectMarkers(grayscale_frame, arucoDict, parameters=arucoParam)

    for n, foundItem in enumerate(corners):

        # Calculate marker size
        cX, cY, diag02, diag13 = marker_visualization(corners, ids, n, grayscale_frame)

        if prev_aruco_navigation_active and ids[n][0] == navigation_list[actual_ID]:
            cv2.line(frame, (416, 312), (cX, cY), (0, 0, 255), 2)

            # Calculating movement
            command = calc_navigation(cX, diag02, diag13, max_marker_size, actual_ID, camera_resolution, frame)

            # Next entry in navigation list
            if command == MovementEnum.SEARCH_NEXT and actual_ID < len(navigation_list):
                actual_ID = (actual_ID + 1)

            # End of navigation list
            if actual_ID == len(navigation_list):
                command = MovementEnum.STOP

    return command
