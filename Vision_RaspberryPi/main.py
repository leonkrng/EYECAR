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

from aruco_navigation.webcam_video_stream import WebcamVideoStream
from aruco_navigation.find_aruco_markers import find_aruco_markers
from aruco_navigation.marker_visualization import marker_visualization 
from aruco_navigation.navigation import navigation
from communication.serial_input import SerialInput

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


#construct the argument parse and parse the arguments
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
        cX, cY, diag02, diag13 = marker_visualization(corners, ids, n)		# Visualisierung des Markers und Berechnung der markanten Längen
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


