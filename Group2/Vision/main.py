import pyrealsense2 as rs
import numpy as np
from pydarknet import Detector, Image
import cv2
import time

if __name__ == "__main__":

    # Realsense initiation
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    net = Detector(bytes("YOLO3-4-Py/cfg/yolov3.cfg", encoding="utf-8"), bytes("YOLO3-4-Py/weights/yolov3.weights", encoding="utf-8"), 0,
                   bytes("YOLO3-4-Py/cfg/coco.data", encoding="utf-8"))

    ###cap = cv2.VideoCapture(0)

    try:
        while True:
            start_time = time.time()
            frames = pipeline.wait_for_frames()
            frame = frames.get_color_frame()
            if not frame:
                continue
            frame = np.asanyarray(frame.get_data())

            # Detection

            dark_frame = Image(frame)
            results = net.detect(dark_frame)
            del dark_frame

            end_time = time.time()
            print("Elapsed Time:",end_time-start_time)

            for cat, score, bounds in results:
                x, y, w, h = bounds
                cv2.rectangle(frame, (int(x-w/2),int(y-h/2)),(int(x+w/2),int(y+h/2)),(255,0,0))
                cv2.putText(frame, str(cat.decode("utf-8")), (int(x), int(y)), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 0))

            cv2.imshow("preview", frame)

            k = cv2.waitKey(1)
            if k == 0xFF & ord("q"):
                break
    finally:
        # Realsense stop
        pipeline.stop()