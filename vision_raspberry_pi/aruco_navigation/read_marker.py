import cv2
import cv2.aruco as aruco

# Reads the marker on the frame and returns the corners and the id.
def read_marker(self, frame_to_read):

    grayscale_frame = cv2.cvtColor(frame_to_read, cv2.COLOR_BGR2GRAY)

    key = getattr(aruco, f"DICT_5X5_50")

    aruco_dict = aruco.getPredefinedDictionary(key)
    aruco_param = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(aruco_dict, aruco_param)

    (corners, ids, rejected) = aruco_detector.detectMarkers(grayscale_frame)

    if ids is None or len(corners) == 0:
        return None, None

    return corners[0][0], ids[0]
