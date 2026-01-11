import cv2

# Draws a border box around the marker
def draw_marker_border(frame, corners, marker_id):

    (tl, tr, br, bl) = corners

    tl = tuple(map(int, tl))
    tr = tuple(map(int, tr))
    br = tuple(map(int, br))
    bl = tuple(map(int, bl))

    cv2.line(frame, tl, tr, (0, 255, 0), 2)
    cv2.line(frame, tr, br, (0, 255, 0), 2)
    cv2.line(frame, br, bl, (0, 255, 0), 2)
    cv2.line(frame, bl, tl, (0, 255, 0), 2)

    cv2.putText(
        frame,
        str(int(marker_id)),
        (tl[0], tl[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        3
    )
