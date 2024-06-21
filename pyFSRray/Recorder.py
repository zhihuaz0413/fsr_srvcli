import FSRray
import numpy as np
import time
import FSRmsg_pb2
import argparse
import os

class Recorder:
    def __init__(self, layers = 2, filename = "fsrdata.bin"):
        self.data = np.zeros((layers, 16, 16))
        
        self.fsrray = FSRray.FSRray(16, layers)
        self.fsrray.set_callback(self.callback)
        self.fsrmsg = FSRmsg_pb2.FSRMsg()
        self.filename = filename

    def callback(self, values, dt):
        # print("dt = {}".format(dt[0]))
        # print("values = {}".format(values))
        # for i in range(16):
        #     print(values[i*16:(i+1)*16])
        fsrdata = FSRmsg_pb2.FSRData()
        fsrdata.timestamp = str(dt[0])
        fsrdata.value = ' '.join(map(str, values))
        #print(fsrdata.__str__())
        self.fsrmsg.fsr_data.append(fsrdata)

    def spin(self):
        # while True:
        self.fsrray.connect()
        time.sleep(5)
        self.fsrray.disconnect()
        # Write the new address book back to disk.
        with open(self.filename, "wb") as f:
            f.write(self.fsrmsg.SerializeToString())

    def parse_msg(self, filename):
        fsrmsg = FSRmsg_pb2.FSRMsg()
        with open(filename, "rb") as f:
            fsrmsg.ParseFromString(f.read())
        return fsrmsg

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script record data from FSR.')
    parser.add_argument('filename', default='fsrdata.bin')           # positional argument
    parser.add_argument('-l', '--layers', type=int, default=2)      # option that takes a value
    path = os.path.dirname(os.path.abspath(__file__)) + '/../data/'
    args = parser.parse_args()
    if not os.path.exists(path):
        os.makedirs(path)
    file_name = path + args.filename
    print("Recording {} layers data in {}".format(args.layers, file_name))
    recorder = Recorder(args.layers, file_name)
    recorder.spin()
    
    # fsrmsg = recorder.parse_msg("fsrdata.bin")
    # print(fsrmsg)


