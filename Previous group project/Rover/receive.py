import time
from digi.xbee.devices import XBeeDevice

class Corr_receive:
    def __init__(self,PORT="COM4",BAUD_RATE=115200):
        self.device= XBeeDevice(PORT, BAUD_RATE)
        self.last_message = None
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
            
            
    def get_message(self, timeout=0.0, max_age=1.0):
        start = time.monotonic()
        while True:
            packet = self.last_message
            now = time.monotonic()
            if packet is not None:
                message, received_at = packet

                if now - received_at <= max_age:
                    return message

            if now - start >= timeout:
                return None

            time.sleep(0.001)
