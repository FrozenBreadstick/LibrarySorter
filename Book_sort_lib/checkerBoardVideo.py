import cv2
import numpy as np

# Open the default camera
cap = cv2.VideoCapture(0)

# Define the dimensions of the checkerboard (number of internal corners per a chessboard row and column)
checkerboard_size = (6, 8)  # Adjust this based on your checkerboard (number of corners per row and column)

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
while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Frame not captured.")
        exit()

    # Convert the image to grayscale
    grayIMG = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Try to find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(grayIMG, checkerboard_size, None)
    
    ### If the checkerboard is found, refine the corners and draw them on the frame
    if ret:
        # Display success message
        print("Checkerboard detected!")

        # Refine the corners
        corners_refined = cv2.cornerSubPix(grayIMG, corners, (11, 11), (-1, -1), criteria)

        # Draw the checkerboard corners on the frame
        cv2.drawChessboardCorners(frame, checkerboard_size, corners_refined, ret)
        # Estimate pose
        success, rvec, tvec = cv2.solvePnP(obj_points, corners_refined, camera_matrix, dist_coeffs)

      

    ### If the checkerboard is not found, display a message
    if not ret:
        # Displaymessage
        print("Checkerboard NOT detected!")

    # Display the frame
    cv2.imshow('Checkerboard Detection', frame)

    # Exit loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()
