import cv2
import numpy as np
import math,time
from sklearn.cluster import DBSCAN
import os # Import os for checking file existence

# Camera Configuration Constants (mm)
# UNUSED AFTER CALIBRATION
FOCAL_LENGTH = 1.0 # 
SENSOR_HEIGHT = 2.4
SENSOR_WIDTH = 3.2
CAM_FOV = 140  # degrees

# Detection Thresholds
MIN_CONTOUR_AREA = 100
CIRCULARITY_THRESHOLD = 0.80
SQUARE_ASPECT_RATIO_MIN = 0.8
SQUARE_ASPECT_RATIO_MAX = 1.4

# Known Object Sizes (mm) - >>> CRITICAL FOR ACCURATE DISTANCE CALIBRATION <<<
# Please measure these precisely in the real world
SINGLE_CIRCLE_SIZE = 70      # Real-world diameter of a single circular bay marker
MULTI_CIRCLE_SIZE = 70       # Real-world diameter of individual circles within a multi-circle group (if applicable)
SQUARE_SIZE = 50             # Real-world side length of a single square picking station marker
CIRCLE_GROUP_SPACING = 100       # Real-world center-to-center distance between two circular bay markers
SQUARE_GROUP_SPACING = 80   # Real-world center-to-center distance between two square picking station markers

# HSV Color Ranges (These ranges should now be more stable due to LAB/CLAHE preprocessing)
# You might need to slightly re-tune these after the lighting normalization.
# These are less critical for 'black' objects as adaptive thresholding is used.
lower_black_pick = np.array([0, 0, 0])
upper_black_pick = np.array([180, 255, 130])
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

lower_white = np.array([0, 0, 150])
upper_white = np.array([80, 55, 255])

lower_green = np.array([65, 200, 95])
upper_green = np.array([85, 255, 190])
        

def compute_distance_and_bearing(bbox, frame_shape, known_size_mm, camera_matrix=None):
    """ Calculate distance and bearing to object based on bounding box.
        Uses camera_matrix for more accurate focal length and principal point.
        
        Args:
            bbox (tuple): (x, y, w, h) of the bounding box in pixels.
            frame_shape (tuple): (height, width, channels) of the image frame.
            known_size_mm (float): The real-world size (e.g., width, height, or diameter) of the object in millimeters.
            camera_matrix (np.array): The 3x3 camera intrinsic matrix from calibration.
            
        Returns:
            tuple: (target_x, target_y, distance_m, bearing_deg)
    """
    x, y, w, h = bbox
    target_x = int(x + w / 2)
    target_y = int(y + h / 2)

    if camera_matrix is not None:
        fx = camera_matrix[0, 0] # Focal length in x direction (pixels)
        fy = camera_matrix[1, 1] # Focal length in y direction (pixels)
        cx = camera_matrix[0, 2] # Principal point x-coordinate
        cy = camera_matrix[1, 2] # Principal point y-coordinate
        
        object_size_px = 0.0
        focal_length_to_use = 0.0

        # Determine which dimension (width or height) to use for object_size_px
        # and its corresponding focal length (fx or fy).
        # This choice is CRITICAL for accurate distance.
        # If the known_size_mm refers to the real-world width, use 'w' and 'fx'.
        # If known_size_mm refers to the real-world height, use 'h' and 'fy'.
        # For objects that are roughly square or circular, using the average or max is often a practical choice.
        
        # Here we use the average of bbox dimensions and average focal length.
        # This is a good general approach but might be less precise than using specific width/height if object orientation is known.
        
        # More robust for varied objects: Use the dimension that best corresponds to known_size_mm
        # If the object is wider than it is tall in the image, use width and fx
        if w > h and w > 0:
            object_size_px = w
            focal_length_to_use = fx
        elif h > 0: # If the object is taller, or width is zero, use height and fy
            object_size_px = h
            focal_length_to_use = fy
        else: # Both w and h are 0, or invalid
            return target_x, target_y, 0.0, 0.0

        if object_size_px == 0 or focal_length_to_use == 0:
            return target_x, target_y, 0.0, 0.0 # Avoid division by zero

        distance_mm = (known_size_mm * focal_length_to_use) / object_size_px
        distance_m = (distance_mm / 1000) / 2 # Magic
        
        # Bearing calculation using camera matrix (relative to the principal point cx)
        # Angle = atan((pixel_offset_from_center) / focal_length_x)
        angle_rad = np.arctan((target_x - cx) / fx)
        bearing_deg = np.degrees(angle_rad)

    else:
        # Fallback to previous calculation if no camera_matrix is provided.
        # This should ideally not be used if calibration is available.
        print("Warning: No camera matrix loaded, using estimated focal length for distance calculation.")
        object_size = max(w, h)
        img_height_px = frame_shape[0]
        if object_size == 0:
            return target_x, target_y, 0.0, 0.0
        distance_mm = (FOCAL_LENGTH * known_size_mm * img_height_px) / (object_size * SENSOR_HEIGHT)
        distance_m = distance_mm / 1000

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
                circles.append((center, radius)) # Store center and radius

                # Draw circle and label
                cv2.circle(frame, center, radius, (0, 255, 0), 2)
                cv2.putText(frame, "Circle", (center[0] - 30, center[1] - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return circles


def detect_squares(contours_black, frame):
    """ Detect square objects from contours """
    square_bboxes = [] # Store (x,y,w,h) for squares

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
                    square_bboxes.append((x, y, w, h))

                    # Draw a rectangle for the square and label
                    cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(frame, "Square", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
    return square_bboxes


def process_circle_groups(circles_data, frame, camera_matrix=None):
    """ Process groups of circles and calculate distance/bearing using DBSCAN clustering """
    if not circles_data:
        return []

    # Extract circle centers for clustering
    circle_centers = np.array([c[0] for c in circles_data])
    
    # Use DBSCAN to cluster circles based on proximity
    # eps=150 is a good starting point for grouping, adjust based on physical spacing in pixels
    clustering = DBSCAN(eps=150, min_samples=1).fit(circle_centers)
    labels = clustering.labels_
    
    group_results = []

    for group_id in np.unique(labels):
        members_indices = np.where(labels == group_id)[0]
        members_centers = circle_centers[members_indices]
        num_circles = len(members_centers)
        
        # Calculate bounding box around the circle group
        min_x = np.min(members_centers[:, 0])
        max_x = np.max(members_centers[:, 0])
        min_y = np.min(members_centers[:, 1])
        max_y = np.max(members_centers[:, 1])
        
        # Add padding to the bounding box
        padding = 20 # Can be tuned
        bbox_x1 = int(min_x - padding)
        bbox_y1 = int(min_y - padding)
        bbox_x2 = int(max_x + padding)
        bbox_y2 = int(max_y + padding)
        
        bbox_width = bbox_x2 - bbox_x1
        bbox_height = bbox_y2 - bbox_y1

        # Determine known size based on number of circles in group (Bay Number Markers)
        object_size_for_distance_px = 0.0
        known_size_mm_for_group = 0.0
        group_label = ""

        if num_circles == 1:
            # For a single circle, use its detected radius * 2 (diameter) for pixel size
            # And use SINGLE_CIRCLE_SIZE for known real-world diameter
            _, radius = circles_data[members_indices[0]]
            object_size_for_distance_px = radius * 2
            known_size_mm_for_group = SINGLE_CIRCLE_SIZE
            group_label = "Bay Marker (1 Circle)"
            
        elif num_circles == 2:
            # For two circles, calculate the pixel distance between their centers.
            # Use CIRCLE_GROUP_SPACING as the known real-world distance between their centers.
            center1_px = members_centers[0]
            center2_px = members_centers[1]
            object_size_for_distance_px = math.dist(center1_px, center2_px)
            known_size_mm_for_group = CIRCLE_GROUP_SPACING # Real-world center-to-center distance
            group_label = "Bay Marker (2 Circles)"
            
        else:  # 3 or more circles
            # For multiple circles, use the extent of the bounding box
            object_size_for_distance_px = max(bbox_width, bbox_height)
            # A generic scaling, adjust if specific pattern is known for 3+
            known_size_mm_for_group = CIRCLE_GROUP_SPACING * (num_circles - 1) 
            group_label = f"Bay Marker ({num_circles} Circles)"

        # Calculate distance and bearing for the group.
        # Create a dummy bbox for compute_distance_and_bearing that reflects our chosen object_size_for_distance_px.
        # We need to map object_size_for_distance_px to a w or h.
        # A robust way is to pass a bbox whose larger dimension is object_size_for_distance_px
        temp_bbox = (bbox_x1, bbox_y1, object_size_for_distance_px, object_size_for_distance_px) # Make it square

        _, _, distance_m, bearing_deg = compute_distance_and_bearing( # +5 is trim
            temp_bbox,
            frame.shape,
            known_size_mm_for_group,
            camera_matrix # Pass camera matrix
        )
        bearing_deg += 5.0 # Trim adjustment

        # Draw bounding box around the circle group
        cv2.rectangle(frame, (bbox_x1, bbox_y1), (bbox_x2, bbox_y2), (0, 255, 0), 2)
        
        # Draw group center
        group_center_x = int(np.mean(members_centers[:, 0]))
        group_center_y = int(np.mean(members_centers[:, 1]))
        cv2.circle(frame, (group_center_x, group_center_y), 10, (0, 255, 0), 2)

        # Add distance/bearing text
        cv2.putText(frame, group_label, (bbox_x1, bbox_y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"{distance_m:.2f}m, {bearing_deg:.1f}deg",
                   (bbox_x1, bbox_y2 + 20),
                   cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        group_results.append((distance_m, bearing_deg))

    return group_results


def process_square_groups(square_bboxes, frame, camera_matrix=None):
    """ Process groups of squares and calculate distance/bearing with bounding boxes around entire groups """
    if not square_bboxes:
        return []
    
    # Extract square centers for clustering
    square_centers = np.array([(b[0] + b[2]//2, b[1] + b[3]//2) for b in square_bboxes])
    
    # Use DBSCAN to cluster squares based on proximity
    # eps=150 is a good starting point for grouping, adjust based on physical spacing in pixels
    clustering = DBSCAN(eps=70, min_samples=1).fit(square_centers)
    labels = clustering.labels_
    
    group_results = []

    for group_id in np.unique(labels):
        members_indices = np.where(labels == group_id)[0]
        members_bboxes = [square_bboxes[i] for i in members_indices]
        members_centers = square_centers[members_indices]
        num_squares = len(members_centers)
        
        # Calculate bounding box around the entire group
        min_x = np.min([b[0] for b in members_bboxes])
        max_x = np.max([b[0] + b[2] for b in members_bboxes])
        min_y = np.min([b[1] for b in members_bboxes])
        max_y = np.max([b[1] + b[3] for b in members_bboxes])
        
        # Add padding to the bounding box
        padding = 15 # Can be tuned
        bbox_x1 = int(min_x - padding)
        bbox_y1 = int(min_y - padding)
        bbox_x2 = int(max_x + padding)
        bbox_y2 = int(max_y + padding)
        
        bbox_width = bbox_x2 - bbox_x1
        bbox_height = bbox_y2 - bbox_y1

        object_size_for_distance_px = 0.0
        known_size_mm_for_group = 0.0
        group_label = ""

        # Determine platform type based on number of squares (Picking Station Markers)
        if num_squares == 1:
            # For a single square, use its bounding box dimensions
            x, y, w, h = members_bboxes[0]
            object_size_for_distance_px = max(w, h)
            known_size_mm_for_group = SQUARE_SIZE # Real-world side length
            group_label = "Picking Station (1 Square)"
        elif num_squares == 2:
            # For two squares, calculate pixel distance between their centers
            center1_px = members_centers[0]
            center2_px = members_centers[1]
            object_size_for_distance_px = math.dist(center1_px, center2_px)
            known_size_mm_for_group = SQUARE_GROUP_SPACING # Real-world center-to-center distance
            group_label = "Picking Station (2 Squares)"
        elif num_squares == 3:
            object_size_for_distance_px = max(bbox_width, bbox_height)
            known_size_mm_for_group = SQUARE_GROUP_SPACING * 2 # Assuming linear arrangement
            group_label = "Picking Station (3 Squares)"
        else:
            object_size_for_distance_px = max(bbox_width, bbox_height)
            known_size_mm_for_group = SQUARE_GROUP_SPACING * (num_squares - 1)
            group_label = f"Picking Station ({num_squares} Squares)"

        # Calculate distance and bearing for the group.
        # Create a dummy bbox for compute_distance_and_bearing that reflects our chosen object_size_for_distance_px.
        temp_bbox = (bbox_x1, bbox_y1, object_size_for_distance_px, object_size_for_distance_px) # Make it square

        _, _, distance_m, bearing_deg = compute_distance_and_bearing( # +5 is trim
            temp_bbox,
            frame.shape,
            known_size_mm_for_group,
            camera_matrix # Pass camera matrix
        )
        bearing_deg += 5 # Trim adjustment

        # Draw bounding box around the entire group
        cv2.rectangle(frame, (bbox_x1, bbox_y1), (bbox_x2, bbox_y2), (255, 0, 0), 2)
        
        # Draw group center point
        group_center_x = int(np.mean(members_centers[:, 0]))
        group_center_y = int(np.mean(members_centers[:, 1]))
        cv2.circle(frame, (group_center_x, group_center_y), 5, (255, 0, 0), -1)

        # Draw group label and distance/bearing
        cv2.putText(frame, group_label, (bbox_x1, bbox_y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(frame, f"{distance_m:.2f}m, {bearing_deg:.1f}deg",
                   (bbox_x1, bbox_y2 + 20),
                   cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        group_results.append((distance_m, bearing_deg))

    return group_results
    

def show_frame(frame):
    cv2.imshow("Frame", frame)
    cv2.waitKey(1)

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
            print("[ Error ] Could not open video stream.")
            exit()
        print("[ OK ] VisionSystem initialised.")

        # --- Calibration Parameters ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration_data()
        self.new_camera_matrix = None # For optimal new camera matrix after undistortion
        self.roi = None # Region of interest for cropping black borders

    def load_calibration_data(self):
        """Loads camera matrix and distortion coefficients from .npy files."""
        # Ensure the 'vision' directory exists and the files are in it
        if os.path.exists("vision/camera_matrix.npy") and os.path.exists("vision/dist_coeffs.npy"):
            try:
                self.camera_matrix = np.load("vision/camera_matrix.npy")
                self.dist_coeffs = np.load("vision/dist_coeffs.npy")
                print("[ OK ] Loaded camera calibration data.")
            except Exception as e:
                print(f"[ ERROR ] Could not load calibration coefficients: {e}")
        else:
            print("[ ERROR ] Camera calibration files (camera_matrix.npy, dist_coeffs.npy) not found in 'vision/' directory.")
            
    def get_items(self):
        return self.items_rb if self.items_rb else [(0,0)]
    def get_packing_stations(self):
        return self.packing_station_rb if self.packing_station_rb else [(0,0)]
    def get_row_markers(self):
        return self.row_marker_rb if self.row_marker_rb else [(0,0)]
    def get_shelves(self):
        return self.shelf_rb if self.shelf_rb else [(0,0)]
    def get_picking_stations(self):
        return self.picking_station_rb if self.picking_station_rb else [(0,0)]
    def get_obstacles(self):
        return self.obstacles_rb if self.obstacles_rb else [(0,0)]
    def get_walls(self):
        return self.walls_rb if self.walls_rb else [(0,0)]
    
    def get_all(self):
        return {
            "items": self.items_rb,
            "packing_stations": self.packing_station_rb,
            "row_markers": self.row_marker_rb,
            "shelves": self.shelf_rb,
            "picking_stations": self.picking_station_rb,
            "obstacles": self.obstacles_rb,
            "walls": self.walls_rb
        }
    
    def camera_release(self):
        self.frame_cap.release()
        cv2.destroyAllWindows()


    def UpdateObjects(self, debug_mode=False):
        """ Main camera operation function"""
        # cleanup return variables
        self.items_rb.clear()
        self.packing_station_rb.clear()
        self.obstacles_rb.clear()
        self.row_marker_rb.clear()
        self.shelf_rb.clear()
        self.picking_station_rb.clear()
        self.walls_rb.clear()
        self.debug_mode = debug_mode

        # Capture and preprocess frame
        ret, frame = self.frame_cap.read()
        if not ret:
            print("[ ERROR ] Failed to grab frame.")
            return

        frame = cv2.resize(frame, (640, 480))
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # --- Undistort the frame if calibration data is available ---
        matrix_for_undistortion = None
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            h, w = frame.shape[:2]
            # Get the optimal new camera matrix and ROI only once
            if self.new_camera_matrix is None or self.roi is None:
                self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
                print("[ OK ] Calibrated vision for undistortion")

            # Undistort
            undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix)
            
            # Crop the image to remove black borders
            x, y, w_roi, h_roi = self.roi
            frame = undistorted_frame[y:y+h_roi, x:x+w_roi]
            
            # Use the new_camera_matrix for distance calculations after undistortion and cropping
            matrix_for_undistortion = self.new_camera_matrix
        else:
            # If no calibration, use the original frame and camera_matrix (which will be None)
            matrix_for_undistortion = None 
        # --- End Undistortion ---

        # --- Lighting Normalization (LAB + CLAHE) ---
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)

        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8,8)) 
        cl = clahe.apply(l_channel)
        limg = cv2.merge((cl,a_channel,b_channel))
        enhanced_bgr = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        hsv = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2HSV)
        # --- End Lighting Normalization ---

        # Create color masks using the ENHANCED HSV image
        mask_orange = cv2.inRange(hsv, lower_orange1, upper_orange1) | cv2.inRange(hsv, lower_orange2, upper_orange2)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # --- Adaptive Thresholding for ALL Black Objects (Bay Number Markers & Picking Station Markers) ---
        gray_enhanced = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2GRAY)

        all_black_mask = cv2.adaptiveThreshold(gray_enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                               cv2.THRESH_BINARY_INV, 81, 50) # TUNE blockSize and C!

        kernel = np.ones((3,3),np.uint8) 
        all_black_mask = cv2.morphologyEx(all_black_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        all_black_mask = cv2.morphologyEx(all_black_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        # --- End Adaptive Thresholding ---

        colour_masks = {
            "Item":     (mask_orange,   (0, 140, 255),  SINGLE_CIRCLE_SIZE), 
            "Ramp": (mask_yellow,   (0, 255, 255),  MULTI_CIRCLE_SIZE), # Use MULTI_CIRCLE_SIZE for platforms or adjust
            # "Shelf":    (mask_blue,     (255, 0, 0),    150), 
            # "Wall":     (mask_white,    (255, 255, 255), 500), 
            # "Obstacle":(mask_green,    (0, 255, 0),    200) 
        }

        for label, (mask, draw_colour, known_size_mm) in colour_masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > MIN_CONTOUR_AREA:
                    x, y, w, h = cv2.boundingRect(contour)

                    M = cv2.moments(contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        cv2.putText(frame, label, (cx-30, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_colour, 2)
        
                    cv2.drawContours(frame, [contour], -1, draw_colour, 2)

                    # ===== DISTANCE AND BEARING =====
                    _, _, distance_m, bearing_deg = compute_distance_and_bearing(
                        (x, y, w, h), 
                        frame.shape, 
                        known_size_mm, 
                        matrix_for_undistortion
                    )

                    text = f"{label}: {distance_m:.2f} m, {bearing_deg:.1f} degrees"
                    cv2.putText(frame, text, (x+w+10, y+h-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

                    # ===== POPULATE RETURN VARIABLES FOR COLOR DETECTION HERE =====
                    if label == "Item": 
                        self.items_rb.append((distance_m, bearing_deg))
                    elif label == "Ramp":
                        self.packing_station_rb.append((distance_m, bearing_deg))
                    elif label == "Shelf":
                        self.shelf_rb.append((distance_m, bearing_deg))
                    elif label == "Obstacle": 
                        self.obstacles_rb.append((distance_m, bearing_deg))
                    elif label == "Wall":
                        self.walls_rb.append((distance_m, bearing_deg))

        # Find contours from the combined black mask for shapes
        contours_black_shapes, _ = cv2.findContours(all_black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Detect circles first (Bay Number Markers)
        circles_data = detect_circles(contours_black_shapes, frame) # Returns list of ((center_x, center_y), radius)
        
        # Detect squares (Picking Station Markers)
        square_bboxes = detect_squares(contours_black_shapes, frame) # Returns list of (x,y,w,h)

        # Process circle groups for row markers (Bay Number Markers)
        circle_group_results = process_circle_groups(circles_data, frame, matrix_for_undistortion)
        if circle_group_results:
            self.row_marker_rb.extend(circle_group_results)

        # Process square groups for picking stations (Picking Station Markers)
        square_group_results = process_square_groups(square_bboxes, frame, matrix_for_undistortion)
        if square_group_results:
            self.picking_station_rb.extend(square_group_results)

        # Add shape count overlay
        if len(circles_data) > 0 or len(square_bboxes) > 0:
            text_circles = f"Detected Circles: {len(circles_data)}"
            text_squares = f"Detected Squares: {len(square_bboxes)}"
            cv2.putText(frame, text_circles, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, text_squares, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)


        # Display results
        # show_frame(frame)
        if self.debug_mode:
            show_frame(frame)
            # cv2.imshow("CLAHE Enhanced Frame", enhanced_bgr)
            # cv2.imshow("Orange Mask (Item)", mask_orange)
            # cv2.imshow("Yellow Mask (Platform)", mask_yellow)
            # cv2.imshow("Blue Mask (Shelf)", mask_blue)
            # cv2.imshow("White Mask (Wall)", mask_white)
            # cv2.imshow("Green Mask (Obstacle)", mask_green)
            # cv2.imshow("All Black Mask (Bay/Picking)", all_black_mask)
            # print("Frame processed")
            

# Example usage (you can add this outside the class or in a main function)
if __name__ == "__main__":
    vs = VisionSystem()
    vs.debug_mode = True # Set to True to see all debug windows

    try:
        while True:
            vs.UpdateObjects()
            # You can access detected objects here:
            # print("Items:", vs.get_items())
            # print("Picking Stations:", vs.get_picking_stations())
            # print("Row Markers (Bay Numbers):", vs.get_row_markers())
            time.sleep(2)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    finally:
        vs.camera_release()