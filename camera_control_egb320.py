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
    frame = cv2.rotate(frame, cv2.ROTATE_180)

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
    lower_green = (65, 200, 95)		    # Lower bound for green in HSV
    upper_green = (85, 255, 190)		# Upper bound for green in HSV
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    lower_black_aisle = (0, 0, 0)
    upper_black_aisle = (179, 182, 110)
    black_mask_aisle = cv2.inRange(hsv_frame, lower_black_aisle, upper_black_aisle)

    lower_black_picking = (0, 0, 0)
    upper_black_picking = (0, 0, 77)
    black_mask_picking = cv2.inRange(hsv_frame, lower_black_picking, upper_black_picking)

    ## Combine Masks ##
    combined_mask = cv2.bitwise_or(orange_mask, blue_mask)
    combined_mask = cv2.bitwise_or(combined_mask, yellow_mask)
    combined_mask = cv2.bitwise_or(combined_mask, green_mask)
    combined_mask = cv2.bitwise_or(combined_mask, black_mask_aisle)
    combined_mask = cv2.bitwise_or(combined_mask, black_mask_picking)

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
                angle_deg = (offset_px / frame.shape[1]) * 140  # 140 is your FOV in degrees
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
                angle_deg = (offset_px / frame.shape[1]) * 140
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
        angle_deg = (offset_px / frame.shape[1]) * 140  # 140 is your FOV in degrees
        cv2.putText(frame, f"Angle: {angle_deg:.1f} deg", (ux, uy - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        #print(f"Shelf angle: {angle_deg:.1f} degrees")

        black_contours_aisle, _ = cv2.findContours(black_mask_aisle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        black_contours_picking, _ = cv2.findContours(black_mask_picking, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        MIN_W = 0
        MAX_W = 30
        filtered_black_contours = []

        # Combine and filter black contours
        for cnt in black_contours_aisle + black_contours_picking:
            x, y, w, h = cv2.boundingRect(cnt)
            if MIN_W < w < MAX_W:
                filtered_black_contours.append(cnt)

        # Only count squares (width ≈ 4.5cm) and circles (diameter ≈ 7cm)
        REAL_SQUARE_WIDTH = 4.5  # cm
        REAL_CIRCLE_DIAM = 7.0   # cm
        SQUARE_TOLERANCE = 0.25  # 25% tolerance
        CIRCLE_TOLERANCE = 0.25  # 25% tolerance

        valid_markers = 0
        for cnt in filtered_black_contours:
            bx, by, bw, bh = cv2.boundingRect(cnt)
            # Estimate square width in cm (use REAL_SQUARE_WIDTH)
            est_square_width = (REAL_SQUARE_WIDTH * FOCAL_CONST) / bw if bw > 0 else 0
            is_square = abs(est_square_width - REAL_SQUARE_WIDTH) / REAL_SQUARE_WIDTH < SQUARE_TOLERANCE

            # Estimate circle diameter in cm (use REAL_CIRCLE_DIAM)
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            est_circle_diam = (REAL_CIRCLE_DIAM * FOCAL_CONST) / (2 * radius) if radius > 0 else 0
            is_circle = abs(est_circle_diam - REAL_CIRCLE_DIAM) / REAL_CIRCLE_DIAM < CIRCLE_TOLERANCE

            if is_square or is_circle:
                valid_markers += 1
                # Draw marker
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 0, 0), 2)
                if is_circle:
                    cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 0, 0), 2)
                # Black Angle
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx_m = int(M["m10"] / M["m00"])
                    frame_center_x = frame.shape[1] // 2
                    offset_px = cx_m - frame_center_x
                    angle_deg = (offset_px / frame.shape[1]) * 140
                    cv2.putText(frame, f"{angle_deg:.1f} deg", (cx_m, by - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Black Distance
                if bw > 0:
                    distance_black = (REAL_SQUARE_WIDTH * FOCAL_CONST) / bw
                    cv2.putText(frame, f"{distance_black:.1f}cm", (bx, by - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Display only valid marker count
        cv2.putText(frame, f"Row Markers: {valid_markers}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)

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
# cv2.waitKey(0)           # Make the program wait until you press a key before continuing.
cv2.destroyAllWindows()  # Close all OpenCV windows
