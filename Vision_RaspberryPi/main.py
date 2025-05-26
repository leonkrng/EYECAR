from __future__ import print_function
from threading import Thread
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import cv2.aruco as aruco
from gpiozero import CPUTemperature
import serial
import numpy as np
import time

import cv2

# 0: Ende der Navigationsliste erreicht. Nicht mehr bewegen
# 1: kein relevanten Marker erkannt
# 2: Fahrzeug nach rechts ausrichten
# 3: Fahrzeug vorwärts bewegen
# 4: Fahrzeug nach links ausrichten
# 5: nächsten Marker suchen
# 6: zu nah am Marker. Nicht mehr bewegen




val = "-"

#xCoor = [None]*4        # Liste fur x-Koordinaten der 4 Marker-Ecken
#yCoor = [None]*4        # Liste für y-Koordinaten der 4 Marker-Ecken

cameraRes = [832, 600]		# Camera resolution [x, y]
naviList = [1, 2, 3, 4]
maxMarkerSize = 0.4			# maximum marker size in relation to camera x-resoulution (0-1)
prevArucoNavigationAktiv = 0 #für Erkennung steigende Flanke
actualID = 0
errorCar = 0
command = -1
textToSend = "0"
splitString = ["-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1","-1"]
#Die folgende Klasse ist für das Einlesen der Kameraframes da
class WebcamVideoStream:
    def __init__(self, src=1):
        # initialize the video camera stream and read the first frame
        # from the stream
        #832x624
        #640480
        #960x640
        #self repräsentiert die Referenz zum aktuell erzeugten Objekt! 
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

####### Die Funktion erkennt anhand des Kamera Frames den Aruco Marker und gibt die Ecken und ID's zurueck ############
def findArucoMarkers(img, markerSize =5 , totalMarkers=50, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key= getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    (corners, ids, rejected) = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)

# drawing markers bounding box and centre + calculate marker position for EYE-Car moving command ################
def markerVisualization(corners, ids, n):
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


####### Navigations-Funktion kann evtl für die Ausrichtung bei der Objekterkennung verwendet werden ##########
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




class SerialInput:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyAMA0',
                                 baudrate=115200,
                                 bytesize=8,
                                 timeout=50,
                                 xonxoff=False,
                                 rtscts=False,
                                 dsrdtr=False)
        self.stopped = False
    
    def start(self):
        # start the thread to read frames from the video cap
        Thread(target=self.readSerialInput, args=()).start()
        Thread(target=self.sendSerialOutput, args=()).start()
        return self
    
    def sendSerialOutput(self):
        msg = 1 
        while True:
            try:
                
                if self.ser.out_waiting == 0:
                    self.ser.write(textToSend.encode('utf-8'))
                #self.ser.write('cool\r\n'.encode('utf-8'))
                else:
                    false
                    #print(self.ser.out_waiting)
                time.sleep(0.1)
            except:
                if msg:
                    print("errorSendSerialOutput")
                    msg = 0
    
    def readSerialInput(self):    
        
        
        global splitString
        #self.ser.flush()
        #self.ser.reset_input_buffer()
        while True:
            if self.stopped:
                return
            serialInputString = str(self.ser.readline())
            #print(serialInputString)
            #val=serialInputString
            
            if len(serialInputString) > 2:   
                try:
                    splitString = serialInputString.split("/")
                    
                except:
                    val="splitfail"
            #cv2.putText(img=frame, text=str(text), org=(50, 50), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 255, 0),thickness=1)
            
           
            
            #print(text)
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        self.ser.close()
# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-n", "--num-frames", type=int, default=500
#help="# of frames to loop over for FPS test")
#ap.add_argument("-d", "--display", type=int, default=1,
#    help="Whether or not frames should be displayed")
#args = vars(ap.parse_args())


# grab a pointer to the video stream and initialize the FPS counter
# created a *threaded* video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from webcam...")

vs = WebcamVideoStream(src=0).start()

fps = FPS().start()
serialp = SerialInput().start()

# loop over some frames...this time using the threaded stream
while 1:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    key= getattr(aruco, f'DICT_5X5_50')

    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    (corners, ids, rejected) = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    
   # arucofound = findArucoMarkers(frame)
    n = 0
    
    for foundItem in corners:
        
        # flatten the ArUco IDs list
        #print("ActualId: "+str(naviList[actualID]))
        #print("n: "+str(n))
        #print("ids[n]:"+str(ids[n]))
        #ids = ids.flatten()
        cX, cY, diag02, diag13 = markerVisualization(corners, ids, n)		# Visualisierung des Markers und Berechnung der markanten Längen
        if prevArucoNavigationAktiv and ids[n][0] == naviList[actualID]:
            cv2.line(frame, (416,312), (cX, cY), (0, 0, 255), 2)
            
            command = navigation(cX, diag02, diag13, maxMarkerSize, actualID, cameraRes)	# Berechnen der Bewegung
            if command == 5 and actualID < len(naviList):	# Wenn der aktuelle Marker erreicht wurde
                actualID = actualID + 1						# Nächsten Index der Navigationsliste als Marker-ID suchen
            if actualID == len(naviList):			        # Sobald das ende der Navigationsliste erreicht wird
                command = 0							        # 0: Ende der Navigationsliste erreicht

        n += 1

    if splitString[10] == '1' and prevArucoNavigationAktiv == 0:
        actualID = 0
        print("Navigation wurde neu gestartet. Bitte beim ersten Marker anfangen")
    
    if splitString[10] == '1':
        prevArucoNavigationAktiv = 1
    else:
        prevArucoNavigationAktiv = 0
        
        
    #print("Command: ",command)
    #print("Navi: ", actualID)
    
    try:
        cv2.rectangle(frame, (15,20), (275,190), (50,50,50), -1) #grauer Hintergrund Telemetriedaten
        cv2.line(frame, (396,312), (436, 312), (255,255,255), 1) #Fadenkreuz waagerechte Linie
        cv2.line(frame, (416,292), (416, 332), (255,255,255), 1) #Fadenkreuz senkrechte Linie
        
        cv2.putText(img=frame, text=str(splitString[2]), org=(420, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1) #Motorstellwerte  
        cv2.putText(img=frame, text=str(splitString[3]), org=(520, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
        cv2.putText(img=frame, text=str(splitString[4]), org=(420, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
        cv2.putText(img=frame, text=str(splitString[5]), org=(520, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
        
        cv2.putText(img=frame, text="Akku:   "+str(splitString[6]), org=(20, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
        cv2.putText(img=frame, text="DC24V: "+str(splitString[7]), org=(20, 65), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
        cv2.putText(img=frame, text="Logik:   "+str(splitString[8]), org=(20, 90), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
        
         
        cv2.putText(img=frame, text="RSSI:", org=(20, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255, 255),thickness=1)    
        rssi = int(int(splitString[9])*1.053 + 130)
        if int(splitString[9]) == -1:
            cv2.putText(img=frame, text=" kein Signal!", org=(80, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 10, 255),thickness=1)
        elif int(splitString[9]) > -100:
            cv2.putText(img=frame, text=str(splitString[9])+"dbm", org=(90, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
            cv2.line(frame, (20,125), (20+100, 125), (150,150,150),2)
            cv2.line(frame, (20,125), (20+rssi, 125), (0,255,0),2)
            cv2.putText(img=frame, text=str(rssi)+"%", org=(200, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
            
        else:
            cv2.line(frame, (20,115), (20+100, 125), (150,150,150),2)
            cv2.line(frame, (20,115), (20+rssi, 125), (0,255,0),2)
            cv2.putText(img=frame, text=str(rssi)+"%", org=(200, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
            
            cv2.putText(img=frame, text=str(splitString[9])+"dbm", org=(90, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 94, 255),thickness=1)
        
        
        cv2.putText(img=frame, text="ArucoNavigation:", org=(20, 150), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
        if prevArucoNavigationAktiv:
            
            cv2.putText(img=frame, text="aktiv", org=(200, 150), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
            if str(splitString[11]) == 's':
                cv2.putText(img=frame, text="Suche Marker "+str(naviList[actualID]), org=(350, 290), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, color=(0, 0, 255),thickness=2)
            else:
                cv2.putText(img=frame, text=str(splitString[11]), org=(408, 290), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 255),thickness=2)
        else:
            cv2.putText(img=frame, text="inaktiv", org=(200, 150), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            



        cv2.putText(img=frame, text="Ultraschallsens.: ", org=(20, 180), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
        if str(splitString[16]) == '1':
            cv2.putText(img=frame, text="aktiv", org=(200, 180), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
        else:
            cv2.putText(img=frame, text="inaktiv", org=(200, 180), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)

        
        #cv2.putText(img=frame, text=str(splitString[12]), org=(450, 300), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)   
        #cv2.putText(img=frame, text=str(splitString[13]), org=(400, 330), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
        #cv2.putText(img=frame, text=str(splitString[14]), org=(500, 330), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
        #cv2.putText(img=frame, text=str(splitString[15]), org=(450, 360), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)

        ##if str(splitString[17]) == '0': #Notaus        1= alles ok  0 = NOT HALT betätigt
        ##    cv2.putText(img=frame, text="NOT-HALT!", org=(300, 250), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.8, color=(0,0,255),thickness=4)

        


    except:
        print("err")
    
    
    
    #frame = imutils.resize(frame, width=700)
    window_name = 'EYE-CAR'
    cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
    cv2.moveWindow(window_name, 0, 36)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN,
                          cv2.WINDOW_FULLSCREEN)
   
    cv2.imshow(window_name, frame)
    textToSend = "1/7/"+str(command)+"/8\r\n"
    #print(textToSend)
    # update the FPS counter
    #fps.update()   
    key = cv2.waitKey(1) & 0xFF    
       
    # If the `q` key was pressed, break from the loop
    
    if key == ord('q'):
        
        cv2.destroyAllWindows()
        break
    
    
    #fps.stop()
    #print(CPUTemperature())
    #print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# stop the timer and display FPS information
#fps.stop()
#print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
#print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
serialp.stop()


