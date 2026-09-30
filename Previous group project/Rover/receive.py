import time
from digi.xbee.devices import XBeeDevice

class Corr_receive:
    def __init__(self,PORT="COM4",BAUD_RATE=115200):
        self.device= XBeeDevice(PORT, BAUD_RATE)
        self.last_message = "0,0,0"
        self.open()
        print('Corr_receive is a Great Success')

    def data_received_callback(self, xbee_message):
        try:
            self.last_message = xbee_message.data.decode('utf-8')
        except UnicodeDecodeError:
            self.last_message = xbee_message.data.hex()

    def open(self): 
        if not self.device.is_open(): 
            self.device.open() 
            self.device.add_data_received_callback(self.data_received_callback) 
            
    def close(self): 
        if self.device.is_open(): 
            self.device.close()

    def get_message(self,timeout=0.05):
        start = time.time()
        initial = self.last_message

        while time.time() - start < timeout:
            if self.last_message != initial:
                return self.last_message
            time.sleep(0.001)
        return initial