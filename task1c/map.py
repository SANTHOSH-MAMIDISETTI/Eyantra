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
    if ids is not None:
        print("Detected Aruco markers with IDs:", ids.flatten())
        return corners, ids
    else:
        raise ValueError("No Aruco markers detected!")

def find_obstacles(transformed_image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)
    
    # Apply a binary threshold to detect obstacles
    _, binary_image = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    
    # Find contours which represent obstacles
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create a mask for the obstacles
    mask = np.zeros_like(transformed_image)
    
    # Draw filled contours on the mask
    cv2.drawContours(mask, contours, -1, (255, 255, 255), thickness=cv2.FILLED)
    
    # Count the number of obstacles and calculate the total area
    num_obstacles = len(contours)
    total_area = sum(cv2.contourArea(contour) for contour in contours)
    
    return mask, num_obstacles, total_area

def save_results(detected_ids, num_obstacles, total_area):
    # Save the results to a .txt file
    with open("output.txt", "w") as f:
        f.write(f"Detected Aruco IDs: {detected_ids}\n")
        f.write(f"Number of Obstacles Detected: {num_obstacles}\n")
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

    # Create a mask with only obstacles
    obstacle_mask, num_obstacles, total_area = find_obstacles(image)

    # Save the obstacle mask as an image
    cv2.imwrite("obstacle_mask.jpg", obstacle_mask)
    print("Obstacle mask saved as obstacle_mask.jpg")

    # Save the results in a text file
    save_results(ids.flatten(), num_obstacles, total_area)
    print(f"Results saved in output.txt")

if __name__ == "__main__":
    main()
