import cv2

# drawing markers bounding box and centre + calculate marker position for EYE-Car moving command ################
def marker_visualization(corners, ids, n, frame):
    # loop over the detected ArUCo corners
    for markerCorner in corners[n]:
        # extract the marker corners (which are always returned
        # in top-left, top-right, bottom-right, and bottom-left
        # order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        # draw the bounding box of the ArUCo detection
        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
        # draw the ArUco marker ID on the frame
        #print(str(ids[n][0]))
        cv2.putText(frame, str(ids[n][0]),
            (topLeft[0], topLeft[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            1, (0, 255, 0), 3)
        # compute and draw the center (x, y)-coordinates of the ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        #compute length of all marker sides														    	 	#	(x0/y0)         (x1/y1)
        aLen = pow(pow(topLeft[0] - topRight[0], 2) + pow(topLeft[1] - topRight[1], 2), 0.5)				#         # # # a # # #
        bLen = pow(pow(bottomRight[0] - topRight[0], 2) + pow(bottomRight[1] - topRight[1], 2), 0.5)		#         #           #
        cLen = pow(pow(bottomRight[0] - bottomLeft[0], 2) + pow(bottomRight[1] - bottomLeft[1], 2), 0.5)	#         d   ArUco   b
        dLen = pow(pow(bottomLeft[0] - topLeft[0], 2) + pow(bottomLeft[1] - topLeft[1], 2), 0.5)			#         #           #
        diag02 = pow(pow(bottomRight[0] - topLeft[0], 2) + pow(bottomRight[1] - topLeft[1], 2), 0.5)		#         # # # c # # #
        diag13 = pow(pow(topRight[0] - bottomLeft[0], 2) + pow(topRight[1] - bottomLeft[1], 2), 0.5)		#    (x3/y3)         (x2/y2)
    return cX, cY, diag02, diag13


