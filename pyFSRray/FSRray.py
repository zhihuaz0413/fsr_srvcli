import serial
import threading
import struct
import logging
import serial.tools.list_ports 
import time
import numpy as np

class FSRray:
    def __init__(self, width=16, nb_layers=1, verbose=False):
        self.array_width = width
        self.nb_layers = nb_layers
        self._callback = None
        self._values = [0] * width * width * nb_layers
        self._dt = [0] * 2
        self._path = None
        self._baud = 500000
        self._timeout = 3
        self._thread = None
        self._running = False
        self._verbose = verbose
        logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO)
        logging.info("FSRray initialized")

    def set_width(self, width):
        self.array_width = width
        self._values = [0] * width * width * self.nb_layers
        logging.info("Array width set to {}".format(width))

    def set_nb_layers(self, nb_layers):
        self.nb_layers = nb_layers
        self._values = [0] * self.array_width * self.array_width * nb_layers
        logging.info("Number of layers set to {}".format(nb_layers))

    def set_callback(self, callback):
        self._callback = callback

    def connect(self, path="/dev/ttyACM0", baud=500000, timeout=3):
        self._path = path
        self._baud = baud
        self._timeout = timeout
        self._thread = threading.Thread(target=self.run)
        self._thread.start()

    def disconnect(self):
        self._running = False
        self._thread.join()

    def read_values(self):
        return self._dt, self._values

    def run(self):
        #connect using vid and pid
        vid = 0x2341
        pid = 0x0042
        error_count = 0
        #find the port_url
        port_url = None
        for port in serial.tools.list_ports.comports():
            if port.vid == vid and port.pid == pid:
                port_url = port.device
                break
        if port_url is None:
            logging.error("No Arduino found")
            return
        
        with serial.Serial(port_url, self._baud, timeout=self._timeout) as arduino:
            dt = [0, 0]
            self._running = True
            time.sleep(1)
            while(self._running):
                try:
                    cmd = bytes([self.array_width])
                    if self.nb_layers == 2:
                        cmd = bytes([self.array_width | 0x80])
                    n = arduino.write(cmd)# send array width
                    #set to 1 the MSB of n if the number of layers is 2
                    for i in range(2):# read two timestamps on 4 bytes each
                        self._dt[i] = struct.unpack('<I', arduino.read(4))[0]
                        logging.debug("dt[{}] = {}".format(i, self._dt[i]))
                    for i in range(self.array_width*self.array_width*self.nb_layers):# read the n*n values on 2 bytes each
                        self._values[i] = struct.unpack('<H', arduino.read(2))[0]
                        logging.debug("values[{}] = {}".format(i, self._values[i]))
                    if self._callback:
                        self._callback(self._values, self._dt)

                except serial.SerialException as e:
                    error_count += 1
                    #logging.error("SerialException: {}".format(e))
                    continue           

if __name__ == "__main__":
    import time
    def callback(values, dt):
        print("dt = {}".format(dt[0]))
        #print 16x16 array
        for i in range(16):
            print(values[i*16:(i+1)*16])
    fsrray = FSRray(16, 2)
    fsrray.set_callback(callback)
    fsrray.connect()
    time.sleep(5)
    fsrray.disconnect()