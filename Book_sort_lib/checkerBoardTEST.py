import cv2
import numpy as np


# Python program to read an write an image
 
import imageio.v2 as imageio
 
# read an image 
###USE RELATIVE PATHS
image = imageio.imread("LibrarySorter/Book_sort_lib/checkerboardEXAMPLE.png")
 

# Set the dimensions of the checkerboard (number of inner corners per row and column)
CHECKERBOARD = (6, 9)  # Example: a 7x7 checkerboard pattern

# Load the image
#image = cv2.imread('')

if image is None:
    print("Error: Image not loaded. Check the file path.")
    # Exit the program
    exit()
else:
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Find the checkerboard corners
ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

if ret:
    # Draw the corners on the image
    cv2.drawChessboardCorners(image, CHECKERBOARD, corners, ret)
    cv2.imshow('Checkerboard Detected', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Checkerboard pattern not found.")



