import cv2
import numpy as np

# Checkerboard dimensions
checkerboard_size = (9, 7)  # Number of inner corners per a chessboard row and column

# Termination criteria for corner refinement (criteria for corner sub-pixel accuracy)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Open the camera
cap = cv2.VideoCapture(0)  # '0' is the default camera. Change it if using an external camera.

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    # If corners are found, refine and draw them
    if ret:
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        cv2.drawChessboardCorners(frame, checkerboard_size, corners_refined, ret)

    # Display the frame
    cv2.imshow('Checkerboard Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
