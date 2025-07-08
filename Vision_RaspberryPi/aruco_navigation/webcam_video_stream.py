from threading import Thread
import cv2

class WebcamVideoStream:
    def __init__(self, src=1):
        # initialize the video camera stream and read the first frame
        # from the stream
        #832x624
        #640480
        #960x640
        #Es werden bei der Initialisierung also die Attribute des gerade erzeugten Objektes beschrieben.
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 832)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,576 )
        (self.grabbed, self.frame) = self.cap.read()
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

        
    def start(self):
        # start the thread to read frames from the video cap
        Thread(target=self.update, args=()).start()
        return self
    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
        
            (self.grabbed, self.frame) = self.cap.read()
            
    def read(self):
        # return the frame most recently read
        return self.frame
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

