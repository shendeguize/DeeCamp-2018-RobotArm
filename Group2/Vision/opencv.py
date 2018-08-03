import pyrealsense2 as rs
import numpy as np
import cv2
import time

if __name__ == "__main__":

    # Realsense initiation
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    ###cap = cv2.VideoCapture(0)

    try:
        while True:
            start_time = time.time()
            frames = pipeline.wait_for_frames()
            frame = frames.get_color_frame()
            if not frame:
                continue
            frame = np.asanyarray(frame.get_data())
            cv2.imshow("1st", frame)

            medianblur = cv2.medianBlur(frame,5)
            gray = cv2.cvtColor(medianblur,cv2.COLOR_BGR2GRAY)

            canny = cv2.Canny(frame,100,200)
            cv2.imshow("2nd", canny)

            circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,10,20,
                                       param1=50,param2=30,minRadius=15,maxRadius=30)

            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
            cv2.imshow('detected circles',frame)

            end_time = time.time()
            print("Elapsed Time:",end_time-start_time)

            k = cv2.waitKey(1)
            if k == 0xFF & ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        # Realsense stop
        pipeline.stop()