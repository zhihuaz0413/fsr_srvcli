import FSRmsg_pb2
import numpy as np
import cv2
fsrmsg = FSRmsg_pb2.FSRMsg()
with open('../data/fsrdataC.bin', "rb") as f:
    fsrmsg.ParseFromString(f.read())
    for data in fsrmsg.fsr_data:
        value = data.value
        ts = data.timestamp
        # print(ts)
        fsr_data = np.fromstring(value, dtype=int, sep=' ')
        fsr_data = fsr_data.reshape(2, 16, 16)
        data_img = fsr_data[0, :, ]
        cv2.imshow('data',data_img)
        cv2.waitKey(1)

