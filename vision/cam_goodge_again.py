import cv2
import numpy as np
import math
from sklearn.cluster import DBSCAN
# from calibration import calibrate

# Camera Configuration Constants (mm)
FOCAL_LENGTH = 2.9
SENSOR_HEIGHT = 2.4
SENSOR_WIDTH = 3.2
CAM_FOV = 140  # degrees

# Detection Thresholds
MIN_CONTOUR_AREA = 150
CIRCULARITY_THRESHOLD = 0.80
SQUARE_ASPECT_RATIO_MIN = 0.9
SQUARE_ASPECT_RATIO_MAX = 1.1

# Known Object Sizes (mm)
SINGLE_CIRCLE_SIZE = 70
MULTI_CIRCLE_SIZE = 100
SQUARE_SIZE = 50
PLATFORM_SPACING = 300 # This might be unused, but keeping it
SQUARE_GROUP_SPACING = 140  # Spacing between squares in a group (mm)

# HSV Color Ranges (These ranges should now be more stable due to LAB/CLAHE preprocessing)
# You might need to slightly re-tune these after the lighting normalization.
lower_black_pick = np.array([0, 0, 0]) # These will be largely replaced by adaptive thresholding
upper_black_pick = np.array([180, 255, 130]) # but kept for potential bitwise AND if needed.
lower_black_aisle = np.array([0, 0, 0])
upper_black_aisle = np.array([180, 100, 130])

lower_orange1 = np.array([0, 120, 80])
upper_orange1 = np.array([15, 255, 255])
lower_orange2 = np.array([170, 120, 80])
upper_orange2 = np.array([180, 255, 255])

lower_yellow = np.array([20, 152, 149])
upper_yellow = np.array([40, 255, 255])

lower_blue = np.array([60, 50, 60])
upper_blue = np.array([150, 255, 255])

lower_white = np.array([0, 0, 150]) # You might need to adjust V and S max for white after CLAHE
upper_white = np.array([80, 55, 255]) # With CLAHE, the V channel might be boosted.

lower_green = np.array([65, 200, 95]) # Need to adjust this for new camera and lighting
upper_green = np.array([85, 255, 190])
        

def compute_distance_and_bearing(bbox, frame_shape, known_size_mm):
    """ Calculate distance and bearing to object based on bounding box"""
    x, y, w, h = bbox
    target_x = int(x + w / 2)
    target_y = int(y + h / 2)

    # Calculate distance using pinhole camera model
    object_size = max(w, h)
    img_height_px = frame_shape[0]
    distance_mm = (FOCAL_LENGTH * known_size_mm * img_height_px) / (object_size * SENSOR_HEIGHT)
    distance_m = distance_mm / 1000

    # Calculate bearing angle from center of frame
    img_width_px = frame_shape[1]
    mid_px = target_x - (img_width_px / 2)
    bearing_deg = (CAM_FOV * mid_px) / img_width_px

    return target_x, target_y, distance_m, bearing_deg


def detect_circles(contours_black, frame): 
    """ Detect circular objects from contours """
    circles = []

    for contour in contours_black:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        if perimeter > 0 and area > MIN_CONTOUR_AREA:
            circularity = (4 * math.pi * area) / (perimeter * perimeter)
            if circularity > CIRCULARITY_THRESHOLD:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                circles.append((center, radius))

                # Draw circle and label
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.putText(frame, "Circle", (center[0] - 30, center[1] - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return circles


def detect_squares(contours_black, frame):
    """ Detect square objects from contours """
    square_centers = []

    for contour in contours_black:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        if perimeter > 0 and area > MIN_CONTOUR_AREA:
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            # A square usually has 4 vertices after approximation. Allow for slight variations (e.g., 4-6)
            if len(approx) >= 4 and len(approx) <= 6:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h

                if SQUARE_ASPECT_RATIO_MIN <= aspect_ratio <= SQUARE_ASPECT_RATIO_MAX:
                    cx_square = x + w // 2
                    cy_square = y + h // 2
                    square_centers.append([cx_square, cy_square])

                # Draw a rectangle for the square and label
                cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(frame, "Square", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
    return square_centers


def process_circle_groups(circles, frame):
    """ Process groups of circles and calculate distance/bearing using DBSCAN clustering """
    if not circles:
        return None

    # Extract circle centers for clustering
    circle_centers = []
    for (center, radius) in circles:
        circle_centers.append([center[0], center[1]])
    
    circle_centers_array = np.array(circle_centers)
    
    # Use DBSCAN to cluster circles based on proximity
    # eps=25 is good for closely spaced circles; adjust if your markers are further apart
    clustering = DBSCAN(eps=250, min_samples=1).fit(circle_centers_array)
    labels = clustering.labels_
    
    # Store results for each group
    group_results = []

    for group_id in np.unique(labels):
        members_array = circle_centers_array[labels == group_id]
        
        # Calculate bounding box around the circle group
        min_x = np.min(members_array[:, 0])
        max_x = np.max(members_array[:, 0])
        min_y = np.min(members_array[:, 1])
        max_y = np.max(members_array[:, 1])
        
        # Add padding to the bounding box
        padding = 30 # Can be tuned
        bbox_x1 = int(min_x - padding)
        bbox_y1 = int(min_y - padding)
        bbox_x2 = int(max_x + padding)
        bbox_y2 = int(max_y + padding)
        
        # Calculate group center and dimensions
        group_center_x = np.mean(members_array[:, 0])
        group_center_y = np.mean(members_array[:, 1])
        bbox_width = bbox_x2 - bbox_x1
        bbox_height = bbox_y2 - bbox_y1

        num_circles = len(members_array)
        
        # Determine known size based on number of circles in group (Bay Number Markers)
        # Assuming single circle is a bay marker, two circles a different type, etc.
        if num_circles == 1:
            # Use a more accurate object size for single circles: the diameter
            circle_idx = np.where(labels == group_id)[0][0] # Get index of the circle in the original list
            _, radius = circles[circle_idx]
            object_size_for_distance = radius * 2 # Diameter
            known_size_mm = SINGLE_CIRCLE_SIZE # Known real-world diameter
            group_label = "Bay Marker (1)"
            
        elif num_circles == 2:
            # Two circles - use distance between centers or bounding box for object_size_for_distance
            # A more robust way might be using the bbox_width or height for known_size_mm based on arrangement
            circle_indices = np.where(labels == group_id)[0]
            center1_px = circle_centers[circle_indices[0]]
            center2_px = circle_centers[circle_indices[1]]
            object_size_for_distance = math.dist(center1_px, center2_px)
            known_size_mm = PLATFORM_SPACING # Or a specific known spacing for two circles
            group_label = "Bay Marker (2)"
            
        else:  # 3 or more circles
            # Multiple circles - use the extent of the bounding box for object_size_for_distance
            object_size_for_distance = max(bbox_width, bbox_height)
            known_size_mm = PLATFORM_SPACING * (num_circles - 1) # A generic scaling, adjust if specific
            group_label = f"Bay Marker ({num_circles})"

        # Calculate distance and bearing for the group using its bounding box dimensions
        # Pass (bbox_x1, bbox_y1, bbox_width, bbox_height) as the bbox
        _, _, distance_m, bearing_deg = compute_distance_and_bearing(
            (bbox_x1, bbox_y1, bbox_width, bbox_height), # Bounding box of the group
            frame.shape,
            known_size_mm
        )

        # Draw bounding box around the circle group
        cv2.rectangle(frame, (bbox_x1, bbox_y1), (bbox_x2, bbox_y2), (0, 255, 0), 2)
        
        # Draw group visualization
        cv2.putText(frame, group_label, (bbox_x1, bbox_y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(frame, (int(group_center_x), int(group_center_y)), 10, (0, 255, 0), 2)

        # Add distance/bearing text
        cv2.putText(frame, f"{distance_m:.2f}m, {bearing_deg:.1f}deg",
                   (bbox_x1, bbox_y2 + 20),
                   cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        group_results.append((distance_m, bearing_deg))

    return group_results # Return list of all detected group results


def process_square_groups(square_centers, frame):
    """ Process groups of squares and calculate distance/bearing with bounding boxes around entire groups """
    if not square_centers:
        return None
    
    # Cluster squares based on proximity (using 25px as clustering distance)
    square_centers_array = np.array(square_centers)
    clustering = DBSCAN(eps=100, min_samples=2).fit(square_centers_array) # Tune eps if squares are further apart
    labels = clustering.labels_
    
    # Store results for each group
    group_results = []

    for group_id in np.unique(labels):
        members_array = square_centers_array[labels == group_id]
        num_squares = len(members_array)
        
        # Calculate bounding box around the entire group
        min_x = np.min(members_array[:, 0])
        max_x = np.max(members_array[:, 0])
        min_y = np.min(members_array[:, 1])
        max_y = np.max(members_array[:, 1])
        
        # Add padding to the bounding box
        padding = 20 # Can be tuned
        bbox_x1 = int(min_x - padding)
        bbox_y1 = int(min_y - padding)
        bbox_x2 = int(max_x + padding)
        bbox_y2 = int(max_y + padding)
        
        # Calculate group center and dimensions
        group_center_x = (bbox_x1 + bbox_x2) / 2
        group_center_y = (bbox_y1 + bbox_y2) / 2
        bbox_width = bbox_x2 - bbox_x1
        bbox_height = bbox_y2 - bbox_y1

        # Determine platform type based on number of squares (Picking Station Markers)
        if num_squares == 1:
            group_label = "Picking Station (1)"
            known_size_mm = SQUARE_SIZE
        elif num_squares == 2:
            group_label = "Picking Station (2)"
            known_size_mm = SQUARE_GROUP_SPACING # Assuming this is the known spacing for 2 squares
        elif num_squares == 3:
            group_label = "Picking Station (3)"
            known_size_mm = SQUARE_GROUP_SPACING * 2 # Assuming this is the known spacing for 3 squares
        else:
            group_label = f"Picking Station ({num_squares})"
            known_size_mm = SQUARE_GROUP_SPACING * (num_squares - 1)

        # Draw bounding box around the entire group
        cv2.rectangle(frame, (bbox_x1, bbox_y1), (bbox_x2, bbox_y2), (255, 0, 0), 2)
        
        # Draw group center point
        cv2.circle(frame, (int(group_center_x), int(group_center_y)), 5, (255, 0, 0), -1)

        # Calculate distance and bearing for the group using the bounding box
        _, _, distance_m, bearing_deg = compute_distance_and_bearing(
            (bbox_x1, bbox_y1, bbox_width, bbox_height),
            frame.shape,
            known_size_mm
        )

        # Draw group label
        cv2.putText(frame, group_label, (bbox_x1, bbox_y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Add distance/bearing text
        cv2.putText(frame, f"{distance_m:.2f}m, {bearing_deg:.1f}deg",
                   (bbox_x1, bbox_y2 + 20),
                   cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        group_results.append((distance_m, bearing_deg))

    return group_results # Return list of all detected group results
    

def show_frame(frame):
    cv2.imshow("Frame", frame)
    cv2.waitKey(1)
# ==========================================================================
# ++   SOME ABSOLUTELY CRACKED OBJECT ORIENTED PROGRAMMING GOING ON HERE  ++
# ==========================================================================

class VisionSystem:
    def __init__(self):
        self.items_rb = [] # Orange
        self.packing_station_rb = [] # Yellow (Platforms)
        self.obstacles_rb = [] # Green
        self.row_marker_rb = [] # Black Circles (Bay Number Markers)
        self.shelf_rb = [] # Blue (Shelves)
        self.picking_station_rb = [] # Black Squares (Picking Station Markers)
        self.walls_rb = [] # White (Walls) - Added a list for walls
        self.debug_mode = False
        self.frame_cap = cv2.VideoCapture(0)
        # Check if camera opened successfully
        if not self.frame_cap.isOpened():
            print("Error: Could not open video stream.")
            exit()
        print("VisionSystem initialised successfully.")

    def get_items(self):
        return self.items_rb
    def get_packing_stations(self):
        return self.packing_station_rb
    def get_row_markers(self):
        return self.row_marker_rb
    def get_shelves(self):
        return self.shelf_rb
    def get_picking_stations(self):
        return self.picking_station_rb
    def get_obstacles(self):
        return self.obstacles_rb
    def get_walls(self): # Added getter for walls
        return self.walls_rb
    
    def camera_release(self):
        self.frame_cap.release()
        cv2.destroyAllWindows()


    def UpdateObjects(self):
        """ Main camera operation function"""
        # cleanup return variables
        self.items_rb.clear()
        self.packing_station_rb.clear()
        self.obstacles_rb.clear()
        self.row_marker_rb.clear()
        self.shelf_rb.clear()
        self.picking_station_rb.clear()
        self.walls_rb.clear() # Clear walls list

        # Capture and preprocess frame
        ret, frame = self.frame_cap.read() # read returns ret and frame
        if not ret:
            print("Failed to grab frame.")
            return # Exit if frame not read correctly

        frame = cv2.resize(frame, (640, 480))
        frame = cv2.rotate(frame, cv2.ROTATE_180) # IF NEEDED, UNCOMMENT THIS IF YOUR CAMERA IS UPSIDE DOWN

        # --- Lighting Normalization (LAB + CLAHE) ---
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        # Apply CLAHE to L-channel to enhance contrast without oversaturation
        # Tune clipLimit (e.g., 2.0-4.0) and tileGridSize (e.g., (8,8) or (10,10))
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8,8)) 
        cl = clahe.apply(l_channel)
        limg = cv2.merge((cl,a_channel,b_channel))
        enhanced_bgr = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        hsv = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2HSV) # This 'hsv' is used for all color masks
        # --- End Lighting Normalization ---

        # Create color masks using the ENHANCED HSV image
        mask_orange = cv2.inRange(hsv, lower_orange1, upper_orange1) | cv2.inRange(hsv, lower_orange2, upper_orange2)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # --- Adaptive Thresholding for ALL Black Objects (Bay Number Markers & Picking Station Markers) ---
        # Convert the enhanced BGR frame to grayscale for adaptive thresholding
        gray_enhanced = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2GRAY)

        # Apply adaptive thresholding to find all dark regions.
        # Tune blockSize (must be odd, e.g., 11, 21, 31) and C (e.g., 2-10) for your specific black markers.
        # THRESH_BINARY_INV means dark pixels become white (255) and light pixels become black (0).
        all_black_mask = cv2.adaptiveThreshold(gray_enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                               cv2.THRESH_BINARY_INV, 81, 50) # Example values, TUNE THESE!

        # Optional: Morphological operations to clean up the mask (e.g., remove noise, close gaps)
        # Adjust kernel size (e.g., (3,3), (5,5)) and iterations based on noise levels
        kernel = np.ones((3,3),np.uint8) 
        all_black_mask = cv2.morphologyEx(all_black_mask, cv2.MORPH_OPEN, kernel, iterations=1) # Remove small specks
        all_black_mask = cv2.morphologyEx(all_black_mask, cv2.MORPH_CLOSE, kernel, iterations=1) # Close small gaps
        # --- End Adaptive Thresholding ---

        colour_masks = {
            "Item":     (mask_orange,   (0, 140, 255),  SINGLE_CIRCLE_SIZE), # Use SINGLE_CIRCLE_SIZE for single item, adjust if multi-item
            "Platform": (mask_yellow,   (0, 255, 255),  MULTI_CIRCLE_SIZE), # Use MULTI_CIRCLE_SIZE for platforms
            "Shelf":    (mask_blue,     (255, 0, 0),    150), # Known size for shelf
            # "Wall":     (mask_white,    (255, 255, 255), 500), # Known size for wall (large object)
            # "Obstacle":(mask_green,    (0, 255, 0),    200) # Known size for obstacle # Not needed for now
        }


        for label, (mask, draw_colour, known_size_mm) in colour_masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > MIN_CONTOUR_AREA: # Use MIN_CONTOUR_AREA
                    # Calculate bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        cv2.putText(frame, label, (cx-30, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_colour, 2)
        
                    # Draw contour (on the original frame for visualization)
                    cv2.drawContours(frame, [contour], -1, draw_colour, 2)

                    # ===== DISTANCE AND BEARING =====
                    _, _, distance_m, bearing_deg = compute_distance_and_bearing((x, y, w, h), frame.shape, known_size_mm)
                    text = f"{label}: {distance_m:.2f} m, {bearing_deg:.1f} degrees"
                    cv2.putText(frame, text, (x+w+5, y+h-5), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

                    # ===== POPULATE RETURN VARIABLES FOR COLOR DETECTION HERE =====
                    if label == "Item": 
                        self.items_rb.append((distance_m, bearing_deg))
                    elif label == "Platform":
                        self.packing_station_rb.append((distance_m, bearing_deg))
                    elif label == "Shelf":
                        self.shelf_rb.append((distance_m, bearing_deg))
                    elif label == "Obstacle": 
                        self.obstacles_rb.append((distance_m, bearing_deg))
                    elif label == "Wall":
                        self.walls_rb.append((distance_m, bearing_deg)) # Populate walls list

        # Find contours from the combined black mask for shapes
        contours_black_shapes, _ = cv2.findContours(all_black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Detect circles first (Bay Number Markers)
        circles = detect_circles(contours_black_shapes, frame)
        circle_count = len(circles)

        # Detect squares (Picking Station Markers)
        square_centers = detect_squares(contours_black_shapes, frame)
        square_count = len(square_centers)

        # Process circle groups for row markers (Bay Number Markers)
        # The process_circle_groups function now returns a list of results
        circle_group_results = process_circle_groups(circles, frame)
        if circle_group_results:
            self.row_marker_rb.extend(circle_group_results) # Extend with all detected groups

        # Process square groups for picking stations (Picking Station Markers)
        # The process_square_groups function now returns a list of results
        square_group_results = process_square_groups(square_centers, frame)
        if square_group_results:
            self.picking_station_rb.extend(square_group_results) # Extend with all detected groups

        # Add shape count overlay
        if circle_count > 0 or square_count > 0:
            text_circles = f"Bay Markers (C): {circle_count}"
            text_squares = f"Picking Stations (S): {square_count}"
            cv2.putText(frame, text_circles, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, text_squares, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)


        # Display results
        show_frame(frame)
        if self.debug_mode:
            # Show original frame, enhanced frame, and individual masks
            cv2.imshow("Original Frame", frame) # Original frame with drawings
            cv2.imshow("CLAHE Enhanced Frame", enhanced_bgr)
            cv2.imshow("Orange Mask (Item)", mask_orange)
            cv2.imshow("Yellow Mask (Platform)", mask_yellow)
            cv2.imshow("Blue Mask (Shelf)", mask_blue)
            cv2.imshow("White Mask (Wall)", mask_white)
            cv2.imshow("Green Mask (Obstacle)", mask_green)
            cv2.imshow("All Black Mask (Bay/Picking)", all_black_mask) # New debug mask
            print("Frame processed")
            
        # You might not need to return anything if the class state is updated directly
        # return (self.packing_station_rb, self.row_marker_rb, self.shelf_rb, self.picking_station_rb)