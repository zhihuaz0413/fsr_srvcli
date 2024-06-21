from fsr_interfaces.srv import Trigger

import rclpy
from rclpy.node import Node

import time
import numpy as np
from pyFSRray import FSRmsg_pb2
from pyFSRray.FSRray import FSRray
from fsr_interfaces.msg import NiForce
import matplotlib.pyplot as plt
import os

from std_msgs.msg import String

from time import sleep
from threading import Thread
 
# function to create threads

class FsrService(Node):
    def __init__(self):
        super().__init__('fsr_service')
        # self.subscription = self.create_subscription(
        #     NiForce,
        #     '/ni_force',
        #     self.ni_daqmx_callback,
        #     10)
        self.niforce = NiForce()
        self.srv = self.create_service(Trigger, 'trigger_recorder', self.fsr_recorder_callback)
        self.duration = 5
        self.fsrmsg = FSRmsg_pb2.FSRMsg()
        self.fsrdata = FSRmsg_pb2.FSRData()
        self.data = np.zeros((2, 16, 16))
        self.show_flag = False
        self.record_flag = False

        self.fsrray = FSRray(16, 2)
        self.fsrray.set_callback(self.fsr_callback)
        self.fsrray.connect()

    def __del__(self):
        self.fsrray.disconnect()

    def fsr_callback(self, values, dt):
        # print("dt = {}.{}".format(dt[0], dt[1]))
        self.fsrdata.timestamp = str(dt[0])
        self.fsrdata.value = ' '.join(map(str, values))
        if self.record_flag:
            self.fsrmsg.fsr_data.append(self.fsrdata)
        #for i in range(16):
        #    print(values[i*16:(i+1)*16])

    def fsr_recorder_callback(self, request, response):
        self.duration = request.duration
        self.show_flag = request.show_flag
        self.record_flag = request.record_flag
        file_name = request.folder + "fsr/"
        if not os.path.exists(file_name):
            os.makedirs(file_name)
        file_name = file_name + request.labels + '_calib.bin' if self.record_flag else file_name + request.labels + '.bin'
        if self.record_flag:
            time.sleep(self.duration)
        self.record_fsrmsg(file_name)
        response.success = True
        response.message = '12345'
        self.get_logger().info('Getting response:\n %d msg: %s' % (response.success, response.message))
        return response
    
    def record_fsrmsg(self, filename):
        with open(filename, "wb") as f:
            f.write(self.fsrmsg.SerializeToString())
        self.get_logger().info('Recorded FSR data in %s' % filename)
        self.fsrmsg.Clear()

    def show_fsrmsg(self, fsrmsg, title):
        len_frames = len(fsrmsg.fsr_data)
        all_layer1 = np.zeros((len_frames,16,16), dtype=np.int32)
        all_layer2 = np.zeros((len_frames,16,16), dtype=np.int32)
        for i in range(len_frames):
            data = fsrmsg.fsr_data[i]
            value = data.value
            ts = data.timestamp
            fsr_data = np.fromstring(value, dtype=int, sep=' ')
            fsr_data = fsr_data.reshape(16, 32)
            all_layer1[i] = fsr_data[:, 0:16]
            all_layer2[i] = fsr_data[:, 16:]
        mean_layer1 = all_layer1.mean(axis=0)
        mean_layer2 = all_layer2.mean(axis=0)
        if self.show_flag:
            #vmin = min(np.min(mean_layer1), np.min(mean_layer2))
            #vmax = max(np.max(mean_layer1), np.max(mean_layer2))
            std_layer1 = all_layer1.std(axis=0)
            std_layer2 = all_layer2.std(axis=0)
            s_vmin = min(np.min(std_layer1), np.min(std_layer2))
            s_vmax = max(np.max(std_layer1), np.max(std_layer2))
            fig, axs = plt.subplots(2, 2, figsize=(6, 6))
            pcm = axs[0, 0].imshow(mean_layer1, cmap='viridis', interpolation='none')#, vmin=vmin, vmax=vmax
            fig.colorbar(pcm, ax=axs[0, 0])
            axs[0, 0].set_title('mean layer1')
            pcm = axs[0, 1].imshow(mean_layer2, cmap='viridis', interpolation='none')#, vmin=vmin, vmax=vmax)
            fig.colorbar(pcm, ax=axs[0, 1])
            axs[0, 1].set_title('mean layer2')
            pcm = axs[1, 0].imshow(std_layer1, cmap='viridis', interpolation='none')#, vmin=s_vmin, vmax=s_vmax)
            fig.colorbar(pcm, ax=axs[1, 0])
            axs[1, 0].set_title('std layer1')
            pcm = axs[1, 1].imshow(std_layer2, cmap='viridis', interpolation='none')#, vmin=s_vmin, vmax=s_vmax)
            fig.colorbar(pcm, ax=axs[1, 1])
            axs[1, 1].set_title('std layer2')
            fig.suptitle(title)
            plt.show()
        return mean_layer1, mean_layer2

def main():
    rclpy.init()
    fsr_service = FsrService()
    rclpy.spin(fsr_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
