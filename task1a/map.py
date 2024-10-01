import cv2
import numpy as np
import argparse
from cv2 import aruco

import cv2
import numpy as np
import argparse

def detect_aruco_markers(image):
    # Load the Aruco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    
    # Detect Aruco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    
    # Check if at least one marker was detected
    if ids is not None:
        print("Detected Aruco markers with IDs:", ids.flatten())
        return corners, ids
    else:
        raise ValueError("No Aruco markers detected!")

def apply_perspective_transform(image, corners):
    # Check if we have exactly 4 corners for Aruco markers
    if len(corners) != 4:
        raise ValueError("Exactly 4 Aruco markers must be detected for perspective transformation.")

    # Extract the corner points and reshape them into the correct format
    src_points = np.array([corner[0][0] for corner in corners], dtype="float32")

    # Ensure the shape of src_points is (4, 2)
    print("Source Points:", src_points)  # For debugging
    print("Shape of Source Points:", src_points.shape)

    # Define the destination points (assuming we want to map to a 500x500 square)
    dst_points = np.array([
        [0, 0],         # Top-left
        [500, 0],       # Top-right
        [500, 500],     # Bottom-right
        [0, 500]        # Bottom-left
    ], dtype="float32")

    print("Destination Points:", dst_points)  # For debugging

    # Ensure both arrays have the right shape and dtype
    assert src_points.shape == (4, 2), "src_points must be 4x2!"
    assert dst_points.shape == (4, 2), "dst_points must be 4x2!"
    
    # Perform the perspective transformation
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    warped_image = cv2.warpPerspective(image, M, (500, 500))  # Adjust the size as needed
    
    return warped_image

def find_obstacles(transformed_image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)
    
    # Apply a binary threshold to detect obstacles
    _, binary_image = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    
    # Find contours which represent obstacles
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Count the number of obstacles
    num_obstacles = len(contours)
    
    # Calculate total area covered by obstacles
    total_area = sum(cv2.contourArea(contour) for contour in contours)
    
    return num_obstacles, total_area

def save_results(detected_ids, num_obstacles, total_area):
    # Save the detected IDs and obstacle data to a .txt file
    with open("output.txt", "w") as f:
        f.write(f"Detected Aruco IDs: {detected_ids}\n")
        f.write(f"Number of Obstacles: {num_obstacles}\n")
        f.write(f"Total Area Covered by Obstacles: {total_area}\n")

def main():
    # Argument parser to take the image path from the command line
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Path to the input image")
    args = parser.parse_args()

    # Load the image
    image = cv2.imread(args.image)
    if image is None:
        raise FileNotFoundError(f"Could not read image {args.image}")

    # Detect Aruco markers
    corners, ids = detect_aruco_markers(image)

    # Apply perspective transformation
    warped_image = apply_perspective_transform(image, corners)

    # Find obstacles in the transformed image
    num_obstacles, total_area = find_obstacles(warped_image)

    # Save the results to a text file
    save_results(ids.flatten(), num_obstacles, total_area)

    # Save the output warped image
    cv2.imwrite("warped_image.jpg", warped_image)
    print("Warped image saved as warped_image.jpg")
    print("Results saved in output.txt")

if __name__ == "__main__":
    main()
