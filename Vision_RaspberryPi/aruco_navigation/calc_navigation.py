from aruco_navigation.movement_enum import MovementEnum
import cv2

def calc_navigation(cX, diag02, diag13, relationX, actualID, resolution, frame):

    # Draw frame where EYECAR drives forward
    centerFrameLeft = int(resolution[0][0]*0.4)
    centerFrameRight = int(resolution[0][0]*0.6)
    cv2.line(frame, [centerFrameLeft, int(resolution[0][1]*0.3)], [centerFrameLeft ,int(resolution[0][1]*0.7)], (0, 0, 255), 1)
    cv2.line(frame, [centerFrameRight, int(resolution[0][1]*0.3)], [centerFrameRight ,int(resolution[0][1]*0.7)], (0, 0, 255), 1)

    # Align EYECAR
    maxSize = relationX * resolution[0][0]

    moveCommand = MovementEnum.NO_MARKER

    if cX < centerFrameLeft:
        moveCommand = MovementEnum.ALIGN_LEFT

    elif cX > centerFrameLeft and cX < centerFrameRight:
        moveCommand = MovementEnum.FORWARD

    elif cX > centerFrameRight:
        moveCommand = MovementEnum.ALIGN_RIGHT

    if diag02 > maxSize or diag13 > maxSize:
        moveCommand = MovementEnum.SEARCH_NEXT    

    return moveCommand


