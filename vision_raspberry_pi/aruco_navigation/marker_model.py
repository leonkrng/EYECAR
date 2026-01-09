# Class that represents the id and geometry of an aruco marker.

class MarkerModel():
    def __init__(self, corners, marker_id):
        self._marker_id = marker_id
        (top_left, top_right, bottom_right, bottom_left) = corners
        # Convert coordiantes to integer
        top_right = (int(top_right[0]), int(top_right[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
        bottom_righ = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        
        # Calculate centerpoint
        self._center_x = int((top_left[0] + bottom_right[0]) / 2.0)
        self._center_y = int((top_left[1] + bottom_right[1]) / 2.0)

        # Calculate diagonals and sides
        self._diag_TL_BR = pow(pow(bottom_right[0] - top_left[0], 2) + pow(bottom_right[1] - top_left[1], 2), 0.5)
        self._diag_TR_BL = pow(pow(top_right[0] - bottom_left[0], 2) + pow(top_right[1] - bottom_left[1], 2), 0.5)

        self._side_TL_BL = pow(pow(bottom_left[0] - top_left[0], 2) + pow(bottom_left[1] - top_left[1], 2), 0.5)
        self._side_TR_BR = pow(pow(bottom_right[0] - top_right[0], 2) + pow(bottom_right[1] - top_right[1], 2), 0.5)

        # Properties
        @property
        def center_x(self):
            return self._center_x

        @property
        def center_y(self):
            return self._center_y

        @property
        def diag_TL_BR(self):
            return self._diag_TL_BR

        @property
        def diag_TR_BL(self):
            return self._diag_TR_BL

        @property
        def side_TL_BL(self):
            return self._side_TL_BL

        @property
        def side_TR_BR(self):
            return self._side_TR_BR

        @property
        def marker_id(self):
            return self._marker_id

