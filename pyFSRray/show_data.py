import glob
import cv2
for img in glob.glob("Path/to/dir/*.jpg"):
    n = cv2.imread(img)
    cv2.imshow('img',n)
    cv2.waitKey(0)