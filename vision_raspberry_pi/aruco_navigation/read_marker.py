import cv2 
import cv2.aruco as aruco
import socket

from aruco_navigation import marker_visualization
from aruco_navigation import calc_navigation
from aruco_navigation import movement_enum

def read_marker(frame, 
                prev_aruco_navigation_active,
                navigation_list,
                max_marker_size,
                camera_resolution,
                actual_ID):

    grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    key = getattr(aruco, f"DICT_5X5_50")

    # Cv2.aruco changed the API with V4.7.0
    if socket.gethostname() != "eye-car-pi":
        arucoDict = aruco.getPredefinedDictionary(key)
        arucoParam = aruco.DetectorParameters()
        arucoDetector = aruco.ArucoDetector(arucoDict,arucoParam) 
        (corners, ids, rejected) = arucoDetector.detectMarkers(grayscale_frame)
    else:
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        (corners, ids, rejected) = aruco.detectMarkers(grayscale_frame, arucoDict, parameters=arucoParam)

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

    return command
