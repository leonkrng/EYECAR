import cv2

# Draws a border box around the marker
def draw_marker_border(frame, corners, id):

    (top_left, top_right, bottom_right, bottom_left) = corners
    cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
    cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
    cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
    cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

    cv2.putText(frame, str(id),
        (top_left[0], top_left[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        1, (0, 255, 0), 3)

