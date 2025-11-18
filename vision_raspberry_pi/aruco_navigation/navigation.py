import cv2

frame = "TODO" # TODO: Platzhalter, definition in main 

def navigation(cX, diag02, diag13, relationX, actualID, *resolution):
    # draw frame where EYE-Car drives forward
    centerFrameLeft = int(resolution[0][0]*0.4)
    centerFrameRight = int(resolution[0][0]*0.6)
    cv2.line(frame, [centerFrameLeft, int(resolution[0][1]*0.3)], [centerFrameLeft ,int(resolution[0][1]*0.7)], (0, 0, 255), 1)
    cv2.line(frame, [centerFrameRight, int(resolution[0][1]*0.3)], [centerFrameRight ,int(resolution[0][1]*0.7)], (0, 0, 255), 1)
    # align EYE-Car
    maxSize = relationX * resolution[0][0]
    moveCommand = 1					# 1: kein relevanten Marker erkannt
    if 1:
        if cX < centerFrameLeft:
            moveCommand = 4			# 4: Fahrzeug nach links ausrichten
        elif cX > centerFrameLeft and cX < centerFrameRight:
            moveCommand = 3			# 3: Fahrzeug vorwärts bewegen
        elif cX > centerFrameRight:
            moveCommand = 2			# 2: Fahrzeug nach rechts ausrichten
        if diag02 > maxSize or diag13 > maxSize:
            moveCommand = 5			# 5: nächsten Marker suchen
    elif diag02 > maxSize or diag13 > maxSize:
        moveCommand = 6				# 6: zu nah am Marker. Nicht mehr bewegen
    else:
        moveCommand = 1				# 1: kein relevanten Marker erkannt
    return moveCommand


