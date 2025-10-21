import serial
import socket
from threading import Thread
import time

class SerialConnection:
    def __init__(self):

        if socket.gethostname() != "eye-car-pi":
            # mock serialport for testing
            self.ser = serial.serial_for_url('loop://',
                                             baudrate=115200,
                                             timeout=1)
        else:
            self.ser = serial.Serial('/dev/ttyAMA0',
                                 baudrate=115200,
                                 bytesize=8,
                                 timeout=50,
                                 xonxoff=False,
                                 rtscts=False,
                                 dsrdtr=False)


        self.stopped = False
        self._text_to_send = "_"
        self._telemtry_data = ["-1"] * 22

    @property
    def text_to_send(self):
        return self._text_to_send

    @text_to_send.setter
    def text_to_send(self, new_text):
        self._text_to_send = new_text

    @property
    def telemetry_data(self):
        return self._telemtry_data 

    @telemetry_data.setter
    def telemetry_data(self, new_data):
        self._telemetry_data = new_data
    
    def start(self):
        try:
            # Start the thread to read frames from the video cap
            Thread(target=self.readSerialInput, args=()).start()
            Thread(target=self.sendSerialOutput, args=()).start()
            return self
        except:
            print("Error starting serial connection threads")
    
    def sendSerialOutput(self):
        msg = 1 
        while True:
            try:
                
                if self.ser.out_waiting == 0:
                    self.ser.write(self.text_to_send.encode('utf-8'))
                else:
                    False
                time.sleep(0.1)
            except:
                if msg:
                    print("errorSendSerialOutput")
                    msg = 0
    
    def readSerialInput(self):

        while True:
            if self.stopped:
                return
            serialInputString = str(self.ser.readline())
            
            if len(serialInputString) > 2:   
                try:
                    self.telemetry_data = serialInputString.split("/")
                    
                except:
                    val="splitfail"           
           
            
    def stop(self):
        # Indicate that the thread should be stopped
        self.stopped = True
        self.ser.close()

