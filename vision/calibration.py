import numpy as np
import cv2
import glob

def calibrate_camera(image_folder="calibration_images", num_corners_row=9, num_corners_col=6, square_size_mm=25):
    """
    Calibrates the camera using chessboard images.

    Args:
        image_folder (str): Path to the folder containing chessboard images.
        num_corners_row (int): Number of inner corners per a chessboard row.
        num_corners_col (int): Number of inner corners per a chessboard column.
        square_size_mm (float): Size of a square on the chessboard in millimeters.

    Returns:
        tuple: (camera_matrix, dist_coeffs) if successful, otherwise (None, None).
    """
    # Define the dimensions of the chessboard
    CHECKERBOARD = (num_corners_row, num_corners_col)

    # Criteria for corner detection
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * square_size_mm # Scale object points by known square size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3D point in real world space
    imgpoints = []  # 2D points in image plane.

    images = glob.glob(f'{image_folder}/*.png') # Assuming .jpg images

    if not images:
        print(f"Error: No images found in '{image_folder}'. Please provide chessboard images.")
        return None, None

    print(f"Found {len(images)} images for calibration.")

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"Warning: Could not read image {fname}, skipping.")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)
        else:
            print(f"Could not find corners in {fname}")

    cv2.destroyAllWindows()

    if not objpoints:
        print("Error: No chessboard corners were detected in any image. Calibration failed.")
        return None, None

    # Perform camera calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        print("\nCalibration successful!")
        print("Camera matrix:\n", mtx)
        print("Distortion coefficients:\n", dist)
        np.save("vision/camera_matrix.npy", mtx)
        np.save("vision/dist_coeffs.npy", dist)
        print("Calibration data saved to camera_matrix.npy and dist_coeffs.npy")
        return mtx, dist
    else:
        print("Calibration failed!")
        return None, None

if __name__ == '__main__':
    # Usage example:
    # 1. Create a folder named 'calibration_images'
    # 2. Place several (e.g., 10-20) images of a chessboard pattern from different angles in it.
    #    Make sure the entire chessboard is visible in most images.
    # 3. Adjust CHECKERBOARD dimensions and square_size_mm to match your physical chessboard.
    #    (e.g., for an 8x6 inner corner chessboard, num_corners_row=8, num_corners_col=6)
    
    # Example for an 9x6 inner corner chessboard with 25mm squares
    mtx, dist = calibrate_camera(image_folder="vision/calibration_images", num_corners_row=7, num_corners_col=5, square_size_mm=10)