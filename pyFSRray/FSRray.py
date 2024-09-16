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
        self.acc = None
        self.fsr = None
        self.dt_acc=0
        self.dt_fsr=0
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

    def connect(self, path="/dev/ttyACM1", baud=500000, timeout=3):
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
        if self._path is None:
            #find the port_url
            port_url = None
            for port in serial.tools.list_ports.comports():
                if port.vid == vid and port.pid == pid:
                    port_url = port.device
                    break
            if port_url is None:
                logging.error("No Arduino found")
                return
            self._path = port_url
        
        with serial.Serial(self._path, self._baud, timeout=self._timeout) as arduino:
            #check if the port is open
            if not arduino.is_open:
                logging.error("Could not open port: {}".format(self._path))
                return
            dt = [0, 0]
            self._running = True
            time.sleep(2)
            #flush the serial port
            arduino.reset_input_buffer()
            arduino.reset_output_buffer()
            mode = 0b11
            size = 16
            cmd = (mode << 6) | size
            cmd_bytes = cmd.to_bytes(1, 'big')
            len = 2*4 + (4*4*3+16*16*2)*2
            while(self._running):
                try:
                    arduino.write(cmd_bytes)
                    d = arduino.read(len)
                    self._dt[0] = int.from_bytes(d[0:4], "little")#accelerometer timestamp
                    self._dt[1] = int.from_bytes(d[4:8], "little")
                    self.acc = np.frombuffer(d[8:8 + 16*6], dtype=np.int16)
                    self.fsr = np.frombuffer(d[8 + 16*6:], dtype=np.uint16)
                    if self._callback:
                        self._callback(self.acc, self.fsr, self._dt)

                except serial.SerialException as e:
                    error_count += 1
                    #logging.error("SerialException: {}".format(e))
                    continue           

if __name__ == "__main__":
    import time
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 " + sys.argv[0] + " <port>")
        sys.exit(1)

    
    
    def callback(acc, fsr, dt):
        print("dt_acc = {}".format(dt[0]))
        print("dt_fsr = {}".format(dt[1]))
        for i in range(16):
            print("acc[{}].z: {}".format(i, acc[i][2]), end=" ")
        print()
        #for i in range(16):
        #     for j in range(16):
        #        print("{}\t".format(fsr[i][1][j]), end=" ")
        #    print()
        print()
        print()
    fsrray = FSRray(16, 2)
    fsrray.set_callback(callback)
    #get the port from the first command line argument
    port = sys.argv[1]
    fsrray.connect(port)
    time.sleep(10)
    fsrray.disconnect()
