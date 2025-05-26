import cv2
import cv2.aruco as aruco

####### Die Funktion erkennt anhand des Kamera Frames den Aruco Marker und gibt die Ecken und ID's zurueck ############
def find_aruco_markers(img, markerSize =5 , totalMarkers=50, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key= getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    (corners, ids, rejected) = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)


