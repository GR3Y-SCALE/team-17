import time
import cv2
import picamera2
import numpy as np


## Camera Object ##
cap = picamera2.Picamera2()
config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(820,616)})
cap.configure(config)
cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
cap.start()

while True:
    ## Displaying Frames ##
    frame = cap.capture_array()
    frame = cv2.resize(frame, (320, 240))
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    ## Colour Spaces ##
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 		# Convert from BGR to HSV colourspace

    # Colour Tracker
    box_w, box_h = 40,40
    middle_x, middle_y = hsv_frame.shape[1] // 2, hsv_frame.shape[0] // 2
    top_left = (middle_x - box_w // 2, middle_y - box_h // 2)
    bottom_right = (middle_x + box_w // 2, middle_y + box_h // 2)
    middle_hsv = hsv_frame[middle_x, middle_y]
    cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)
    print(f"HSV at middle ({middle_x}, {middle_y}): {middle_hsv}")

    ## Orange - Item ##
    lower_orange = (0, 45, 100)		    # Lower bound for orange in HSV
    upper_orange = (225, 250, 255)		# Upper bound for orange in HSV
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

    ## Blue - Shelves ##
    lower_blue = (100, 150, 0)		    # Lower bound for blue in HSV
    upper_blue = (140, 255, 255)		# Upper bound for blue in HSV
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

    ## Yellow - Bay ##
    lower_yellow = (20, 100, 100)		# Lower bound for yellow in HSV
    upper_yellow = (30, 255, 255)		# Upper bound for yellow in HSV
    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

    ## Green - Person ##
    lower_green = (40, 100, 100)		# Lower bound for green in HSV
    upper_green = (80, 255, 255)		# Upper bound for green in HSV
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    ## Combine Masks ##
    combined_mask = cv2.bitwise_or(orange_mask, blue_mask)
    combined_mask = cv2.bitwise_or(combined_mask, yellow_mask)
    combined_mask = cv2.bitwise_or(combined_mask, green_mask)

    # # Print which color masks are detected
    # if np.any(orange_mask):
    #     print("Orange detected")
    # if np.any(blue_mask):
    #     print("Blue detected")
    # if np.any(yellow_mask):
    #     print("Yellow detected")
    # if np.any(green_mask):
    #     print("Green detected")

    # Display the masks
    cv2.imshow("Orange Mask", orange_mask)
    cv2.imshow("Blue Mask", blue_mask)
    cv2.imshow("Yellow Mask", yellow_mask)
    cv2.imshow("Green Mask", green_mask)
    cv2.imshow("Combined Mask", combined_mask)
    cv2.imshow("CameraImage", frame)     # Display the obtained frame in a window called "CameraImage"

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# To save the frame as an image file, uncomment the following line:
# cv2.imwrite("frame0001.png", frame) 

## Cleanup ##
## cv2.waitKey(0)           # Make the program wait until you press a key before continuing.
cap.close()
cv2.destroyAllWindows()  # Close all OpenCV windows

