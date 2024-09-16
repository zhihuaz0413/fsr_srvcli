from fsr_interfaces.srv import Trigger
from fsr_interfaces.srv import StopSig

import rclpy
import rclpy.clock
from rclpy.node import Node

import time
import numpy as np
from pyFSRray import FSRmsg_pb2
from pyFSRray.FSRray import FSRray
import matplotlib.pyplot as plt
import os

import rclpy.time

from time import sleep
from threading import Thread
 
# function to create threads

class FsrService(Node):
    def __init__(self):
        super().__init__('fsr_service')
        self.sub_node = rclpy.create_node('sub_node')
        self.srv = self.create_service(Trigger, 'trigger_recorder', self.fsr_recorder_callback)
        self.cli = self.sub_node.create_client(StopSig, 'stop_signal')
        #while not self.cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('service not available, waiting again...')
        self.duration = 5
        self.fsrmsg = FSRmsg_pb2.FSRMsg()
        self.show_flag = False
        self.record_flag = False
        self.mean_layer1 = None
        self.std_layer1 = None
        self.calib_flag = False
        self.mean_value = 0
        self.baseline_mean_value = 0
        self.fsrray = FSRray(16, 2)
        self.fsrray.set_callback(self.fsr_callback)
        self.fsrray.connect()

    def __del__(self):
        self.fsrray.disconnect()

    def send_request(self):
        print('Sending request')
        self.cli.wait_for_service()
        print('Service is available ---------')
        self.req = StopSig.Request()
        self.req.mean_val = int(self.mean_value)
        self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self.sub_node, self.future)
        # return self.future.result()

    def fsr_callback(self, acc, fsr, dt):
        fsrdata = FSRmsg_pb2.FSRData()
        fsrdata.timestamp = str(time.time_ns())
        fsrdata.fsr = ' '.join(map(str, fsr.flatten().tolist()))
        fsrdata.acc = ' '.join(map(str, acc.flatten().tolist()))
        layer = np.zeros((16, 16), dtype=np.int32)
        fsr_data = np.fromstring(fsrdata.fsr, dtype=int, sep=' ')
        fsr_data = fsr_data.reshape(16, 32)
        layer = fsr_data[:, 0:16]
        self.mean_value = np.mean(layer) - self.baseline_mean_value
        # print('Current Value: - ', np.mean(layer)) 
        # print('Diff Value: - ', self.mean_value)
        if self.calib_flag and self.mean_value > 7:
            self.send_request()
            self.calib_flag = False
        #print(fsrdata.__str__())
        if self.record_flag:
            self.fsrmsg.fsr_data.append(fsrdata)

    def fsr_recorder_callback(self, request, response):
        self.duration = request.duration
        self.show_flag = request.show_flag
        self.record_flag = request.record_flag
        file_name = request.folder + "fsr/"
        calib_file = file_name + "calib/"
        print('request.record_flag: ', request.record_flag)
        if not os.path.exists(file_name):
            os.makedirs(file_name)
        if not os.path.exists(calib_file):
            os.makedirs(calib_file)
        file_name = calib_file + request.labels + '_calib.bin' if self.record_flag else file_name + request.labels + '.bin'
        if self.record_flag:
            time.sleep(self.duration)
            self.mean_layer1, self.std_layer1 = self.calibrate_fsr(self.fsrmsg)
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

    def calibrate_fsr(self, fsrmsg):
        len_frames = len(fsrmsg.fsr_data)
        all_layer1 = np.zeros((len_frames,16,16), dtype=np.int32)
        for i in range(len_frames):
            data = fsrmsg.fsr_data[i]
            fsr = data.fsr
            ts = data.timestamp
            fsr_data = np.fromstring(fsr, dtype=int, sep=' ')
            fsr_data = fsr_data.reshape(16, 32)
            all_layer1[i] = fsr_data[:, 0:16]
        mean_layer1 = all_layer1.mean(axis=0)
        self.baseline_mean_value = np.mean(mean_layer1)
        print('Recorded Baseline Value !!!!!!!!!!!!!!!!! - ', self.baseline_mean_value)
        std_layer1 = all_layer1.std(axis=0)
        self.calib_flag = True
        return mean_layer1, std_layer1

    def show_fsrmsg(self, fsrmsg, title):
        len_frames = len(fsrmsg.fsr_data)
        all_layer1 = np.zeros((len_frames,16,16), dtype=np.int32)
        all_layer2 = np.zeros((len_frames,16,16), dtype=np.int32)
        for i in range(len_frames):
            data = fsrmsg.fsr_data[i]
            fsr = data.fsr
            ts = data.timestamp
            fsr_data = np.fromstring(fsr, dtype=int, sep=' ')
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
