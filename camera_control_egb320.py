import time
import cv2
# import picamera2
import numpy as np
import math

## Camera Object ##
# cap = picamera2.Picamera2() now using 120 USB camera
frame_cap = cv2.VideoCapture(0)
# config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(820,616)})
# cap.configure(config)
# cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,2.0)})
# cap.start()

while True:
    ## Displaying Frames ##
    # frame = cap.capture_array()
    frame = frame_cap.read()[1]
    frame = cv2.resize(frame, (320, 240))
    #frame = cv2.rotate(frame, cv2.ROTATE_180)

    ## Colour Spaces ##
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 		# Convert from BGR to HSV colourspace

    ###### May need to include blur to soften shapes ######

    ## Orange - Item ##
    lower_orange = (10, 230, 170)       # Lower bound for orange in HSV
    upper_orange = (80, 255, 255)		# Upper bound for orange in HSV
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

    ## Blue - Shelves ##
    lower_blue = (90, 230, 70)		    # Lower bound for blue in HSV
    upper_blue = (120, 255, 255)		# Upper bound for blue in HSV
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

    ## Yellow - Bay ##
    lower_yellow = (10, 170, 230)		# Lower bound for yellow in HSV
    upper_yellow = (40, 210, 255)		# Upper bound for yellow in HSV
    yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)

    ## Green - Person ##
    lower_green = (65, 200, 95)		# Lower bound for green in HSV
    upper_green = (85, 255, 190)		# Upper bound for green in HSV
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)


    lower_black = (20, 50, 50)
    upper_black = (30, 160, 120)
    black_mask = cv2.inRange(hsv_frame, lower_black, upper_black)

    ## Combine Masks ##
    combined_mask = cv2.bitwise_or(orange_mask, blue_mask)
    combined_mask = cv2.bitwise_or(combined_mask, yellow_mask)
    combined_mask = cv2.bitwise_or(combined_mask, green_mask)
    combined_mask = cv2.bitwise_or(combined_mask, black_mask)

    # # Print which color masks are detected
    # if np.any(orange_mask):
    #     print("Orange detected")
    # if np.any(blue_mask):
    #     print("Blue detected")
    # if np.any(yellow_mask):
    #     print("Yellow detected")
    # if np.any(green_mask):
    #     print("Green detected")

#############################################

    CAM_FOV = math.radians(140)
    FORWARD_DIR = 0
    FOCAL_CONST = 235

    # Orange Colour Tracking - Object
    orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if orange_contours:
        largest_orange = max(orange_contours, key=cv2.contourArea)
        ox, oy, ow, oh = cv2.boundingRect(largest_orange)
        cv2.rectangle(frame, (ox, oy), (ox + ow, oy + oh), (0, 140, 255), 2)

        # Orange Angle
        for cnt in orange_contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                # Calculate pixel offset from center
                frame_center_x = frame.shape[1] // 2
                offset_px = cx - frame_center_x
                angle_deg = (offset_px / frame.shape[1]) * 66  # 66 is your FOV in degrees
                cv2.putText(frame, f"{angle_deg:.1f} deg", (cx, oy - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)

        # Orange Distance
        REAL_ORANGE_WIDTH = 5.0  # adjust width
        if ow > 0:
            distance_orange = (REAL_ORANGE_WIDTH * FOCAL_CONST) / ow
            cv2.putText(frame, f"{distance_orange:.1f}cm", (ox, oy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
            # print(f"Estimated distance to orange: {distance_orange:.1f} cm")
 
    # Green Colour Tracking - Person
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if green_contours:
        largest_green = max(green_contours, key=cv2.contourArea)
        gx, gy, gw, gh = cv2.boundingRect(largest_green)
        cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), (0, 255, 0), 2)

        # Green Angle
        for cnt in green_contours:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                frame_center_x = frame.shape[1] // 2
                offset_px = cx - frame_center_x
                angle_deg = (offset_px / frame.shape[1]) * 66
                cv2.putText(frame, f"{angle_deg:.1f} deg", (cx, gy - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Green Distance
        REAL_GREEN_WIDTH = 6.0  # adjust width
        if gw > 0:
            distance_green = (REAL_GREEN_WIDTH * FOCAL_CONST) / gw
            cv2.putText(frame, f"{distance_green:.1f}cm", (gx, gy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # print(f"Estimated distance to green: {distance_green:.1f} cm")

    # Blue Colour Tracking - Shelf Angle
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if blue_contours:
        largest_blue = max(blue_contours, key=cv2.contourArea)
        ux, uy, uw, uh = cv2.boundingRect(largest_blue)
        cv2.rectangle(frame, (ux, uy), (ux + uw, uy + uh), (255, 0, 0), 2)

        # Find center of bounding box
        shelf_center_x = ux + uw // 2
        frame_center_x = frame.shape[1] // 2
        offset_px = shelf_center_x - frame_center_x
        # Estimate angle (in degrees) using FOV and pixel offset
        angle_deg = (offset_px / frame.shape[1]) * 66  # 66 is your FOV in degrees
        cv2.putText(frame, f"Angle: {angle_deg:.1f} deg", (ux, uy - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        print(f"Shelf angle: {angle_deg:.1f} degrees")

        black_contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if black_contours:
            largest_black = max(black_contours, key=cv2.contourArea)
            bx, by, bw, bh = cv2.boundingRect(largest_black)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 0, 0), 2)

        num_circle_contours = len(black_contours)
        num_circle_contours = 0
        for cnt in black_contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            if area < 10:
                (x,y), radius = cv2.minEnclosingCircle(cnt)
                circle_area = math.pi * (radius ** 2)
                if area / circle_area > 0.7:
                    num_circle_contours += 1
            # print(f"Number of circle-like contours: {num_circle_contours}")
        
            # Shape Detection (Circle vs Square)
            # print(len(approx))
            # if len(approx) == 1:
            #     shape = "Circle"
            #     print("Circle")
            # else:
            #     shape = "Square"
            #     print("Square")
            # cv2.drawContours(frame, approx, 0, (0, 0, 0), 2)
            # x, y = approx[0][0]
            # cv2.putText(frame, shape, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)

            # Black Angle
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                frame_center_x = frame.shape[1] // 2
                offset_px = cx - frame_center_x
                angle_deg = (offset_px / frame.shape[1]) * 66
                cv2.putText(frame, f"{angle_deg:.1f} deg", (cx, by - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            # Black Distance
            REAL_BLACK_WIDTH = 5.0  # adjust width
            if bw > 0:
                distance_black = (REAL_BLACK_WIDTH * FOCAL_CONST) / bw
                cv2.putText(frame, f"{distance_black:.1f}cm", (bx, by - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                print(f"Estimated distance to black: {distance_black:.1f} cm")

#############################################

    # Display the masks
    # cv2.imshow("Orange Mask", orange_mask)
    # cv2.imshow("Blue Mask", blue_mask)
    # cv2.imshow("Yellow Mask", yellow_mask)
    # cv2.imshow("Green Mask", green_mask)
    # cv2.imshow("Combined Mask", combined_mask)
    cv2.imshow("CameraImage", frame)     # Display the obtained frame in a window called "CameraImage"

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# To save the frame as an image file, uncomment the following line:
# cv2.imwrite("frame0001.png", frame) 

## Cleanup ##
## cv2.waitKey(0)           # Make the program wait until you press a key before continuing.
cv2.destroyAllWindows()  # Close all OpenCV windows
