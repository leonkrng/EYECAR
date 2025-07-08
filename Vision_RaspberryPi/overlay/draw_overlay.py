import cv2 

def draw_overlay(frame, telemetry_data, navigation_list, prev_aruco_navigation_active):
    try:
            cv2.rectangle(frame, (15,20), (275,190), (50,50,50), -1) #grauer Hintergrund Telemetriedaten
            cv2.line(frame, (396,312), (436, 312), (255,255,255), 1) #Fadenkreuz waagerechte Linie
            cv2.line(frame, (416,292), (416, 332), (255,255,255), 1) #Fadenkreuz senkrechte Linie
        
            cv2.putText(img=frame, text=str(telemetry_data[2]), org=(420, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1) #Motorstellwerte  
            cv2.putText(img=frame, text=str(telemetry_data[3]), org=(520, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            cv2.putText(img=frame, text=str(telemetry_data[4]), org=(420, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            cv2.putText(img=frame, text=str(telemetry_data[5]), org=(520, 60), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
        
            cv2.putText(img=frame, text="Akku:   "+str(telemetry_data[6]), org=(20, 40), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
            cv2.putText(img=frame, text="DC24V: "+str(telemetry_data[7]), org=(20, 65), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
            cv2.putText(img=frame, text="Logik:   "+str(telemetry_data[8]), org=(20, 90), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
        
         
            cv2.putText(img=frame, text="RSSI:", org=(20, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255, 255),thickness=1)    
            rssi = int(int(telemetry_data[9])*1.053 + 130)
            if int(telemetry_data[9]) == -1:
                cv2.putText(img=frame, text=" kein Signal!", org=(80, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 10, 255),thickness=1)
            elif int(telemetry_data[9]) > -100:
                cv2.putText(img=frame, text=str(telemetry_data[9])+"dbm", org=(90, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)
                cv2.line(frame, (20,125), (20+100, 125), (150,150,150),2)
                cv2.line(frame, (20,125), (20+rssi, 125), (0,255,0),2)
                cv2.putText(img=frame, text=str(rssi)+"%", org=(200, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
            
            else:
                cv2.line(frame, (20,115), (20+100, 125), (150,150,150),2)
                cv2.line(frame, (20,115), (20+rssi, 125), (0,255,0),2)
                cv2.putText(img=frame, text=str(rssi)+"%", org=(200, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
            
                cv2.putText(img=frame, text=str(telemetry_data[9])+"dbm", org=(90, 120), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 94, 255),thickness=1)
        
        
            cv2.putText(img=frame, text="ArucoNavigation:", org=(20, 150), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            if prev_aruco_navigation_active:
            
                cv2.putText(img=frame, text="aktiv", org=(200, 150), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
                if str(telemetry_data[11]) == 's':
                    cv2.putText(img=frame, text="Suche Marker "+str(navigation_list[actualID]), org=(350, 290), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.9, color=(0, 0, 255),thickness=2)
                else:
                    cv2.putText(img=frame, text=str(telemetry_data[11]), org=(408, 290), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 0, 255),thickness=2)
            else:
                cv2.putText(img=frame, text="inaktiv", org=(200, 150), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            



            cv2.putText(img=frame, text="Ultraschallsens.: ", org=(20, 180), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255, 255, 255),thickness=1)
            if str(telemetry_data[16]) == '1':
                cv2.putText(img=frame, text="aktiv", org=(200, 180), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(0, 255, 0),thickness=1)
            else:
                cv2.putText(img=frame, text="inaktiv", org=(200, 180), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=(255,255,255),thickness=1)


    except:
            print("err")

    # Set window properties
    window_name = "EYE-CAR"
    cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
    cv2.moveWindow(window_name, 0, 36)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    cv2.imshow(window_name, frame)
