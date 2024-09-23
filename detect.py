# import cv2
# import numpy as np

# # Load the image
# image = cv2.imread('1.jpg', cv2.IMREAD_GRAYSCALE)

# # Convert to binary using a threshold
# _, binary_image = cv2.threshold(image, 7, 255, cv2.THRESH_BINARY)

# # Save the binary image
# cv2.imwrite('binary_image.jpg', binary_image)

# # Detect ArUco markers
# aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)  # Choose dictionary
# parameters = cv2.aruco.DetectorParameters_create()

# # Detect markers
# corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(binary_image, aruco_dict, parameters=parameters)

# # Draw detected markers
# image_with_markers = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)  # Convert back to BGR for color drawing
# cv2.aruco.drawDetectedMarkers(image_with_markers, corners, ids)

# # Save the image with detected markers
# cv2.imwrite('detected_markers.jpg', image_with_markers)

# # Display the images
# cv2.imshow('Original Image', image)
# cv2.imshow('Binary Image', binary_image)
# cv2.imshow('Detected Markers', image_with_markers)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

import cv2
import numpy as np

# Load the image
image = cv2.imread('1.jpg', cv2.IMREAD_GRAYSCALE)

# Convert to binary using a threshold
_, binary_image = cv2.threshold(image, 6, 255, cv2.THRESH_BINARY)

# Save the binary image
cv2.imwrite('binary_image.jpg', binary_image)

# Detect ArUco markers
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)  # Choose dictionary
parameters = cv2.aruco.DetectorParameters_create()

# Detect markers
corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(binary_image, aruco_dict, parameters=parameters)

# Draw detected markers
image_with_markers = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)  # Convert back to BGR for color drawing
cv2.aruco.drawDetectedMarkers(image_with_markers, corners, ids)

# Save the image with detected markers
cv2.imwrite('detected_markers.jpg', image_with_markers)

# Display the images
cv2.imshow('Original Image', image)
cv2.imshow('Binary Image', binary_image)
cv2.imshow('Detected Markers', image_with_markers)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Print detected markers and rejected points to the console
if ids is not None:
    for i, corner in enumerate(corners):
        print(f"Marker ID: {ids[i][0]}, Corners: {corner}")

if rejectedImgPoints is not None:
    for i, point in enumerate(rejectedImgPoints):
        print(f"Rejected points for marker {i}:")
        for p in point:
            coords = (int(p[0][0]), int(p[0][1]))
            print(f"Point: {coords}")
