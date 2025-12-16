import cv2 
import cv2.aruco as aruco
import socket

from aruco_navigation import marker_visualization
from aruco_navigation import calc_navigation
from aruco_navigation import movement_enum

def read_marker(frame, 
                prev_aruco_navigation_active=0,
                navigation_list=[1, 2, 3, 4],
                max_marker_size=0.4,
                camera_resolution=[832, 600],
                actual_ID=0):

    grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    key = getattr(aruco, f"DICT_5X5_50")

    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters()
    arucoDetector = aruco.ArucoDetector(arucoDict,arucoParam) 
    (corners, ids, rejected) = arucoDetector.detectMarkers(grayscale_frame)

    command = movement_enum.MovementEnum.NO_MARKER

    #if enumerate(corners) is None:
    #    return command

    for n, foundItem in enumerate(corners):

        # Calculate marker size
        cX, cY, diag02, diag13 = marker_visualization.marker_visualization(corners, ids, n, frame)


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

            else:
                command = MovementEnum.NO_MARKER

    return (command, frame)
