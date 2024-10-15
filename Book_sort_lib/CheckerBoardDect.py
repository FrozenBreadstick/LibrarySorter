import cv2
import numpy as np

# Checkerboard dimensions
checkerboard_size = (6,8)  # Number of inner corners per a chessboard row and column

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D points of the checkerboard corners)
# The coordinates are defined based on the real-world size of the squares
square_size = 0.025  # Size of a square in meters (e.g., 2.5 cm)
obj_points = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
obj_points[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2) * square_size

# Camera parameters (you may need to calibrate your camera for better results)
# Here, we assume a hypothetical camera intrinsics. Replace these with your actual calibration values.
camera_matrix = np.array([[800, 0, 320],  # fx, 0, cx
                           [0, 800, 240],  # 0, fy, cy
                           [0, 0, 1]])     # 0, 0, 1

dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Open the camera
cap = cv2.VideoCapture(0)  # '0' is the default camera

while True:
    ret, frame = cap.read()
    if not ret:
        break
    pass

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    # If corners are found, refine and draw them
    if ret:
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(frame, checkerboard_size, corners_refined, ret)

        # Estimate pose
        success, rvec, tvec = cv2.solvePnP(obj_points, corners_refined, camera_matrix, dist_coeffs)

        if success: 
            # Draw axes
            axis_length = 0.1  # Length of the axes
            axis = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]]).reshape(-1, 3)
            img_points, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)

            # Draw the axes on the image
            frame = cv2.line(frame, tuple(img_points[0].ravel()), tuple(img_points[1].ravel()), (255, 0, 0), 5)  # X axis in red
            frame = cv2.line(frame, tuple(img_points[0].ravel()), tuple(img_points[2].ravel()), (0, 255, 0), 5)  # Y axis in green
            frame = cv2.line(frame, tuple(img_points[0].ravel()), tuple(img_points[3].ravel()), (0, 0, 255), 5)  # Z axis in blue

    # Display the frame
    cv2.imshow('Checkerboard Detection with Pose', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()

