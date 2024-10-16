import cv2
import numpy as np

# Open the default camera
cap = cv2.VideoCapture(0)

# Define the dimensions of the checkerboard (number of internal corners per a chessboard row and column)
checkerboard_size = (6, 8)  # Adjust this based on your checkerboard (number of corners per row and column)

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Frame not captured.")
        break

    # Convert the image to grayscale
    grayIMG = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Try to find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(grayIMG, checkerboard_size, None)

    if ret:
        # Refine the corners
        corners_refined = cv2.cornerSubPix(grayIMG, corners, (11, 11), (-1, -1), criteria)

        # Draw the checkerboard corners on the frame
        cv2.drawChessboardCorners(frame, checkerboard_size, corners_refined, ret)

        # Display success message
        print("Checkerboard detected!")
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
