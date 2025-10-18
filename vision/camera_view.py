import cv2,time


frame_cap = cv2.VideoCapture(0)

while True:
    frame = frame_cap.read()[1]
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    cv2.imshow("Frame", frame)
    cv2.waitKey(500)
    time.sleep(1)