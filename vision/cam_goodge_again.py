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
MIN_CONTOUR_AREA = 200
CIRCULARITY_THRESHOLD = 0.85
SQUARE_ASPECT_RATIO_MIN = 0.9
SQUARE_ASPECT_RATIO_MAX = 1.1

# Known Object Sizes (mm)
SINGLE_CIRCLE_SIZE = 70
MULTI_CIRCLE_SIZE = 120
SQUARE_SIZE = 50
PLATFORM_SPACING = 300
SQUARE_GROUP_SPACING = 140  # Spacing between squares in a group (mm)

# HSV Color Ranges
lower_black_pick = np.array([0, 0, 0]) # Need to slightly adjust this as it's unstable
upper_black_pick = np.array([180, 255, 130])
# upper_black_pick = np.array([180, 255, 150])
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


def detect_circles(contours_black, frame): # Changed from contours to contours_black
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


def detect_squares(contours_black, frame): # Changed from contours to contours_black
    """ Detect square objects from contours """
    square_centers = []

    for contour in contours_black:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        if perimeter > 0 and area > MIN_CONTOUR_AREA:
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            if len(approx) > 3:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h

                if SQUARE_ASPECT_RATIO_MIN <= aspect_ratio <= SQUARE_ASPECT_RATIO_MAX:
                    cx_square = x + w // 2
                    cy_square = y + h // 2
                    square_centers.append([cx_square, cy_square])

                # Draw small marker for individual squares (optional - can be removed)
                # cv2.circle(frame, (cx_square, cy_square), 3, (0, 0, 255), -1)

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
    # eps=80 for circles (smaller than squares since circles are typically closer together)
    clustering = DBSCAN(eps=25, min_samples=1).fit(circle_centers_array)
    labels = clustering.labels_
    
    for group_id in np.unique(labels):
        members_array = circle_centers_array[labels == group_id]
        group_center_x = np.mean(members_array[:, 0])
        group_center_y = np.mean(members_array[:, 1])
        
        num_circles = len(members_array)
        
        # Determine object size and known size based on number of circles in group
        if num_circles == 1:
            # Single circle - use radius * 2 as object size
            circle_idx = np.where(labels == group_id)[0][0]
            _, radius = circles[circle_idx]
            object_size = radius * 2
            known_size_mm = SINGLE_CIRCLE_SIZE
            group_label = "Aisle Marker 1"
            
        elif num_circles == 2:
            # Two circles - use distance between centers
            circle_indices = np.where(labels == group_id)[0]
            center1 = circle_centers[circle_indices[0]]
            center2 = circle_centers[circle_indices[1]]
            object_size = math.dist(center1, center2)
            known_size_mm = MULTI_CIRCLE_SIZE
            group_label = "Aisle Marker 2"
            
        else:  # 3 or more circles
            # Multiple circles - use average distance between all pairs
            circle_indices = np.where(labels == group_id)[0]
            distances = []
            for i in range(len(circle_indices)):
                for j in range(i + 1, len(circle_indices)):
                    center_i = circle_centers[circle_indices[i]]
                    center_j = circle_centers[circle_indices[j]]
                    distances.append(math.dist(center_i, center_j))
            object_size = np.mean(distances) if distances else 50  # fallback
            known_size_mm = MULTI_CIRCLE_SIZE
            group_label = "Aisle Marker 3"

        # Calculate bounding box around the circle group
        min_x = np.min(members_array[:, 0])
        max_x = np.max(members_array[:, 0])
        min_y = np.min(members_array[:, 1])
        max_y = np.max(members_array[:, 1])
        
        # Add padding to the bounding box
        padding = 30
        bbox_x1 = int(min_x - padding)
        bbox_y1 = int(min_y - padding)
        bbox_x2 = int(max_x + padding)
        bbox_y2 = int(max_y + padding)
        
        # Calculate distance and bearing for the group
        target_x, target_y = int(group_center_x), int(group_center_y)
        
        img_height_px = frame.shape[0]
        distance_mm = (FOCAL_LENGTH * known_size_mm * img_height_px) / (object_size * SENSOR_HEIGHT)
        distance_m = distance_mm / 1000

        img_width_px = frame.shape[1]
        pixels_from_center = target_x - (img_width_px / 2)
        bearing_deg = (CAM_FOV * pixels_from_center) / img_width_px

        # Draw bounding box around the circle group
        cv2.rectangle(frame, (bbox_x1, bbox_y1), (bbox_x2, bbox_y2), (0, 255, 0), 2)
        
        # Draw group visualization
        cv2.putText(frame, group_label, (bbox_x1, bbox_y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.circle(frame, (target_x, target_y), 10, (0, 255, 0), 2)

        # Add distance/bearing text
        cv2.putText(frame, f"{distance_m:.2f}m, {bearing_deg:.1f}deg",
                   (bbox_x1, bbox_y2 + 20),
                   cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        return distance_m, bearing_deg

    return None


def process_square_groups(square_centers, frame):
    """ Process groups of squares and calculate distance/bearing with bounding boxes around entire groups """
    if not square_centers:
        return None
    
    # Cluster squares based on proximity (using 25px as clustering distance)
    square_centers_array = np.array(square_centers)
    clustering = DBSCAN(eps=25, min_samples=1).fit(square_centers_array)
    labels = clustering.labels_
    
    for group_id in np.unique(labels):
        members_array = square_centers_array[labels == group_id]
        num_squares = len(members_array)
        
        # Calculate bounding box around the entire group
        min_x = np.min(members_array[:, 0])
        max_x = np.max(members_array[:, 0])
        min_y = np.min(members_array[:, 1])
        max_y = np.max(members_array[:, 1])
        
        # Add padding to the bounding box
        padding = 20
        bbox_x1 = int(min_x - padding)
        bbox_y1 = int(min_y - padding)
        bbox_x2 = int(max_x + padding)
        bbox_y2 = int(max_y + padding)
        
        # Calculate group center and dimensions
        group_center_x = (bbox_x1 + bbox_x2) / 2
        group_center_y = (bbox_y1 + bbox_y2) / 2
        bbox_width = bbox_x2 - bbox_x1
        bbox_height = bbox_y2 - bbox_y1

        # Determine platform type based on number of squares
        if num_squares == 1:
            group_label = "Platform Marker 1"
            known_size_mm = SQUARE_SIZE
        elif num_squares == 2:
            group_label = "Platform Marker 2"
            known_size_mm = SQUARE_GROUP_SPACING
        elif num_squares == 3:
            group_label = "Platform Marker 3"
            known_size_mm = SQUARE_GROUP_SPACING * 2
        else:
            group_label = f"Platform Marker {num_squares}"
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

        return distance_m, bearing_deg
    

    return None

def show_frame(frame):
    cv2.imshow("Frame", frame)
    cv2.waitKey(1)
# ==========================================================================
# ++   SOME ABSOLUTELY CRACKED OBJECT ORIENTED PROGRAMMING GOING ON HERE  ++
# ==========================================================================

class VisionSystem:
    def __init__(self):
        self.items_rb = []
        self.packing_station_rb = []
        self.obstacles_rb = []
        self.row_marker_rb = []
        self.shelf_rb = []
        self.picking_station_rb = []
        self.debug_mode = True
        self.frame_cap = cv2.VideoCapture(0)
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
    
    def camera_release(self):
        self.frame_cap.release()


    def UpdateObjects(self):
        """ Main camera operation function"""
        # cleanup return variables
        self.items_rb.clear()
        self.packing_station_rb.clear()
        self.obstacles_rb.clear()
        self.row_marker_rb.clear()
        self.shelf_rb.clear()
        self.picking_station_rb.clear()

        # Capture and preprocess frame
        frame = self.frame_cap.read()[1]

        frame = cv2.resize(frame, (640, 480))
        frame = cv2.rotate(frame, cv2.ROTATE_180) # IF NEEDED
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create color masks
        mask_black_pick = cv2.inRange(hsv, lower_black_pick, upper_black_pick)
        mask_black_aisle = cv2.inRange(hsv, lower_black_aisle, upper_black_aisle)
        # mask_black = cv2.inRange(hsv, lower_black_aisle, upper_black_aisle)       
        mask_orange = cv2.inRange(hsv, lower_orange1, upper_orange1) | cv2.inRange(hsv, lower_orange2, upper_orange2)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        colour_masks = {
            "Object":   (mask_orange,   (0, 140, 255),  70),
            "Platform": (mask_yellow,   (0, 255, 255),  120),
            "Shelf":    (mask_blue,     (255, 0, 0),    150),
            # "Walls":    (mask_white,    (255, 255, 255), 500),
            "Obstacles":(mask_green,    (0, 255, 0),    200)
        }


        for label, (mask, draw_colour, known_size_mm) in colour_masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > 500:
                    cv2.drawContours(frame, [contour], -1, draw_colour, 2)
                    x, y, w, h = cv2.boundingRect(contour)

                    M = cv2.moments(contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])
                        cv2.putText(frame, label, (cx-30, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_colour, 2)
        
                    # ===== DISTANCE AND BEARING =====
                    _, _, distance_m, bearing_deg = compute_distance_and_bearing((x, y, w, h), frame.shape, known_size_mm)
                    text = f"{label}: {distance_m:.2f} m, {bearing_deg:.1f} degrees"
                    cv2.putText(frame, text, (x+w+5, y+h-5), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

        # ===== RETURN VARIABLES FOR COLOR DETECTION =====
        if label == "Platform":
            self.packing_station_rb.append((distance_m, bearing_deg))
        elif label == "Shelf":
            self.shelf_rb.append((distance_m, bearing_deg))

        # Find contours in black mask
        contours_black_aisle, _ = cv2.findContours(mask_black_aisle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_black_pick, _ = cv2.findContours(mask_black_pick, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Detect circles first
        circles = detect_circles(contours_black_aisle, frame)
        circle_count = len(circles)

        # Only detect squares if no circles found
        square_centers = []
        square_count = 0
        if circle_count == 0:
            square_centers = detect_squares(contours_black_pick, frame)
            square_count = len(square_centers)

        # Process circle groups for row markers
        if circles:
            circle_result = process_circle_groups(circles, frame)
            if circle_result:
                distance_m, bearing_deg = circle_result
                if len(circles) == 1:
                    self.row_marker_rb = [(distance_m, bearing_deg), None, None]
                elif len(circles) == 2:
                    self.row_marker_rb = [None, (distance_m, bearing_deg), None]
                elif len(circles) >= 3:
                    self.row_marker_rb = [None, None, (distance_m, bearing_deg)]

        # Process square groups for picking stations
        if square_centers:
            square_result = process_square_groups(square_centers, frame)
            if square_result:
                distance_m, bearing_deg = square_result
                if len(square_centers) == 1:
                    self.picking_station_rb = [(distance_m, bearing_deg), None, None]
                elif len(square_centers) == 2:
                    self.picking_station_rb = [None, (distance_m, bearing_deg), None]
                elif len(square_centers) >= 3:
                    self.picking_station_rb = [None, None, (distance_m, bearing_deg)]

        # Add shape count overlay
        if circle_count > 0 or square_count > 0:
            text = f"Circles: {circle_count}" if circle_count > 0 else f"Squares: {square_count}"
            # Add black outline for better visibility
            # cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 8, cv2.LINE_AA)
            # cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        # Display results
        show_frame(frame)
        if self.debug_mode:
            combined_mask = mask_orange | mask_yellow | mask_blue | mask_black_pick | mask_black_aisle | mask_white | mask_green
            cv2.imshow("Debug Masks", combined_mask)
            print("Frame processed")
        # cv2.imshow("Undistorted Frame", calibrate.undistort(frame))  # Check that this works


        # cv2.destroyAllWindows()
        # return (self.packing_station_rb, self.row_marker_rb, self.shelf_rb, self.picking_station_rb) # no items? No obstacles, for now