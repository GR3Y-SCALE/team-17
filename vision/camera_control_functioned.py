import time
import cv2
# import picamera2
import numpy as np
import math

# Constants
CAM_FOV = math.radians(140)
FORWARD_DIR = 0
FOCAL_CONST = 235
REAL_ORANGE_WIDTH = 5.0
REAL_GREEN_WIDTH = 6.0
REAL_BLACK_WIDTH = 5.0
FOV_DEGREES = 140

def initialize_camera():
    """Initialize and configure the camera."""
    # cap = picamera2.Picamera2() now using 120 USB camera
    frame_cap = cv2.VideoCapture(0)
    # config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(820,616)})
    # cap.configure(config)
    # cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,2.0)})
    # cap.start()
    return frame_cap

def process_frame(frame_cap):
    """Capture and process a single frame from the camera."""
    # frame = cap.capture_array()
    frame = frame_cap.read()[1]
    frame = cv2.resize(frame, (320, 240))
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    return frame

def create_color_masks(frame):
    """Create color masks for different objects."""
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Orange - Item
    lower_orange = (10, 230, 170)
    upper_orange = (80, 255, 255)
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

    # Blue - Shelves
    lower_blue = (90, 230, 70)
    upper_blue = (120, 255, 255)
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

    # Green - Person
    lower_green = (65, 200, 95)
    upper_green = (85, 255, 190)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Black - Aisle markers
    lower_black_aisle = (0, 0, 0)
    upper_black_aisle = (179, 182, 110)
    black_mask_aisle = cv2.inRange(hsv_frame, lower_black_aisle, upper_black_aisle)

    # Black - Picking markers
    lower_black_picking = (0, 0, 0)
    upper_black_picking = (0, 0, 77)
    black_mask_picking = cv2.inRange(hsv_frame, lower_black_picking, upper_black_picking)

    # Combine masks
    combined_mask = cv2.bitwise_or(orange_mask, blue_mask)
    combined_mask = cv2.bitwise_or(combined_mask, green_mask)
    combined_mask = cv2.bitwise_or(combined_mask, black_mask_aisle)
    combined_mask = cv2.bitwise_or(combined_mask, black_mask_picking)

    return {
        'orange': orange_mask,
        'blue': blue_mask,
        'green': green_mask,
        'black_aisle': black_mask_aisle,
        'black_picking': black_mask_picking,
        'combined': combined_mask
    }

def calculate_angle_from_center(cx, frame_width):
    """Calculate angle in degrees from frame center."""
    frame_center_x = frame_width // 2
    offset_px = cx - frame_center_x
    angle_deg = (offset_px / frame_width) * FOV_DEGREES
    return angle_deg

def calculate_distance(real_width, pixel_width):
    """Calculate distance using focal length formula."""
    if pixel_width > 0:
        return (real_width * FOCAL_CONST) / pixel_width
    return 0

def track_orange_objects(frame, orange_mask):
    """Track orange objects and return their properties."""
    orange_contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not orange_contours:
        return None
    
    largest_orange = max(orange_contours, key=cv2.contourArea)
    ox, oy, ow, oh = cv2.boundingRect(largest_orange)
    cv2.rectangle(frame, (ox, oy), (ox + ow, oy + oh), (0, 140, 255), 2)

    # Calculate angle and distance
    for cnt in orange_contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            angle_deg = calculate_angle_from_center(cx, frame.shape[1])
            cv2.putText(frame, f"{angle_deg:.1f} deg", (cx, oy - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)

    distance_orange = calculate_distance(REAL_ORANGE_WIDTH, ow)
    cv2.putText(frame, f"{distance_orange:.1f}cm", (ox, oy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
    
    return {'distance': distance_orange, 'bounding_box': (ox, oy, ow, oh)}

def track_green_objects(frame, green_mask):
    """Track green objects (person) and return their properties."""
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not green_contours:
        return None
    
    largest_green = max(green_contours, key=cv2.contourArea)
    gx, gy, gw, gh = cv2.boundingRect(largest_green)
    cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), (0, 255, 0), 2)

    # Calculate angle and distance
    for cnt in green_contours:
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            angle_deg = calculate_angle_from_center(cx, frame.shape[1])
            cv2.putText(frame, f"{angle_deg:.1f} deg", (cx, gy - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    distance_green = calculate_distance(REAL_GREEN_WIDTH, gw)
    cv2.putText(frame, f"{distance_green:.1f}cm", (gx, gy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return {'distance': distance_green, 'bounding_box': (gx, gy, gw, gh)}

def track_blue_objects(frame, blue_mask):
    """Track blue objects (shelves) and return their properties."""
    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not blue_contours:
        return None
    
    largest_blue = max(blue_contours, key=cv2.contourArea)
    ux, uy, uw, uh = cv2.boundingRect(largest_blue)
    cv2.rectangle(frame, (ux, uy), (ux + uw, uy + uh), (255, 0, 0), 2)

    # Find center of bounding box and calculate angle
    shelf_center_x = ux + uw // 2
    angle_deg = calculate_angle_from_center(shelf_center_x, frame.shape[1])
    cv2.putText(frame, f"Angle: {angle_deg:.1f} deg", (ux, uy - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    
    return {'angle': angle_deg, 'bounding_box': (ux, uy, uw, uh)}

def process_black_markers(frame, black_mask_aisle, black_mask_picking):
    """Process black markers for navigation."""
    black_contours_aisle, _ = cv2.findContours(black_mask_aisle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    black_contours_picking, _ = cv2.findContours(black_mask_picking, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    MIN_W = 0
    MAX_W = 30
    filtered_black_contours = []

    # Filter aisle contours
    for cnt in black_contours_aisle:
        x, y, w, h = cv2.boundingRect(cnt)
        if MIN_W < w < MAX_W:
            filtered_black_contours.append(cnt)
    
    if filtered_black_contours:
        largest_black = max(filtered_black_contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(largest_black)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 0, 0), 2)
    
    # Filter picking contours
    for cnt in black_contours_picking:
        x, y, w, h = cv2.boundingRect(cnt)
        if MIN_W < w < MAX_W:
            filtered_black_contours.append(cnt)
    
    if filtered_black_contours:
        largest_black = max(filtered_black_contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(largest_black)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 0, 0), 2)

    # Process each filtered contour
    num_circle_contours = 0
    for cnt in filtered_black_contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(cnt)
        if area < 10:
            (x,y), radius = cv2.minEnclosingCircle(cnt)
            circle_area = math.pi * (radius ** 2)
            if area / circle_area > 0.7:
                num_circle_contours += 1

        # Calculate angle
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            angle_deg = calculate_angle_from_center(cx, frame.shape[1])
            cv2.putText(frame, f"{angle_deg:.1f} deg", (cx, by - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Calculate distance
        distance_black = calculate_distance(REAL_BLACK_WIDTH, bw)
        cv2.putText(frame, f"{distance_black:.1f}cm", (bx, by - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    return {'circle_contours': num_circle_contours, 'filtered_contours': len(filtered_black_contours)}

def main():
    """Main function to run the camera control system."""
    frame_cap = initialize_camera()
    
    try:
        while True:
            # Process frame
            frame = process_frame(frame_cap)
            
            # Create color masks
            masks = create_color_masks(frame)
            
            # Track different objects
            orange_data = track_orange_objects(frame, masks['orange'])
            green_data = track_green_objects(frame, masks['green'])
            blue_data = track_blue_objects(frame, masks['blue'])
            black_data = process_black_markers(frame, masks['black_aisle'], masks['black_picking'])
            
            # Display the frame
            cv2.imshow("CameraImage", frame)
            
            # Check for quit condition
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        cleanup(frame_cap)

def cleanup(frame_cap):
    """Clean up resources."""
    frame_cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


