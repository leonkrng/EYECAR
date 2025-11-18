import serial
from threading import Thread
import time

textToSend = "TODO" # TODO: Aus main kopiert: Methode muss angepasst werden

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

