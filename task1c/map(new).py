import cv2
import numpy as np
import argparse
from cv2 import aruco

def detect_aruco_markers(image):
    # Load the Aruco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()

    # Detect Aruco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    # Check if at least one marker was detected
    if ids is not None and len(ids) >= 4:
        print("Detected Aruco markers with IDs:", ids.flatten())
        return corners, ids
    else:
        raise ValueError("Exactly 4 Aruco markers must be detected!")

def apply_perspective_transform(image, corners, ids):
    # Check if we have exactly 4 Aruco markers
    if len(corners) != 4:
        raise ValueError("Exactly 4 Aruco markers must be detected for perspective transformation.")

    # Sort the markers by their IDs to assign them to specific corners
    print(corners)
    sorted_indices = np.argsort(ids.flatten())  # Sort by Aruco IDs
    sorted_corners = [corners[i][0] for i in sorted_indices]  # Rearrange corners based on sorted IDs
    
    # Define the source points from the sorted corners
    src_points = np.array([
        sorted_corners[0][0],  # Aruco marker ID 0 -> top-left
        sorted_corners[1][1],  # Aruco marker ID 1 -> top-right
        sorted_corners[2][2],  # Aruco marker ID 2 -> bottom-right
        sorted_corners[3][3]   # Aruco marker ID 3 -> bottom-left
    ], dtype="float32")

    # Check if src_points has exactly 4 corners with shape (4, 2)
    if src_points.shape != (4, 2):
        raise ValueError(f"src_points shape error: expected (4, 2), got {src_points.shape}")

    # Define the destination points for the warped image (based on sorted Aruco IDs):
    dst_points = np.array([
        [0, 0],         # Top-left
        [1000, 0],      # Top-right
        [1000, 1000],   # Bottom-right
        [0, 1000]       # Bottom-left
    ], dtype="float32")

    # Perform the perspective transformation
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    warped_image = cv2.warpPerspective(image, M, (1000, 1000))  # Warp to a 1000x1000 square image

    return warped_image, ids[sorted_indices]  # Return the warped image and sorted IDs

def find_obstacles(transformed_image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)

    # Apply a binary threshold to isolate the obstacles (assumed to be dark grey)
    _, binary_image = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)  # Inverted to detect dark regions

    # Find contours that represent obstacles
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a copy of the transformed image to draw green contours
    contour_image = transformed_image.copy()

    # Draw the contours in green (RGB: 0, 255, 0) with a thickness of 2 (outline)
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), thickness=2)

    # Count obstacles and calculate total obstacle area
    num_obstacles = len(contours)
    total_area = sum(cv2.contourArea(contour) for contour in contours)

    return contour_image, num_obstacles, total_area

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

    # Apply perspective transformation with the Aruco IDs at the corners
    warped_image, sorted_ids = apply_perspective_transform(image, corners, ids)

    # Save the warped image
    cv2.imwrite("warped_image.jpg", warped_image)
    print("Warped image saved as warped_image.jpg")

    # Find obstacles in the warped image (transformed image) and draw green outlines
    contour_image, num_obstacles, total_area = find_obstacles(warped_image)

    # Save the results to a text file
    save_results(sorted_ids.flatten(), num_obstacles, total_area)

    # Save the output image with obstacle outlines
    cv2.imwrite("obstacle_image.jpg", contour_image)
    print("Obstacle outline image saved as obstacle_image.jpg")
    print("Results saved in output.txt")

if __name__ == "__main__":
    main()

