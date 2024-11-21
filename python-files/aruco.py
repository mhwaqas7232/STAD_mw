import cv2
import numpy as np
import time
# Load the image

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

# Create the ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
# Detect the markers

while(True):
    image = cv2.imread('frame.png')
    time.sleep(0.4)
# Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)
# Print the detected markers
    print("Detected markers:", ids)
    marker_length = 0.1  # Adjust this based on your marker size

    if ids is not None:
    # Prepare the object points (marker corners in 3D)
        object_points = np.array([
            [-marker_length / 2, -marker_length / 2, 0],  # Bottom left
            [marker_length / 2, -marker_length / 2, 0],   # Bottom right
            [marker_length / 2, marker_length / 2, 0],    # Top right
            [-marker_length / 2, marker_length / 2, 0]     # Top left
        ], dtype=np.float32)

    # Create a default camera matrix based on the image size
        height, width = gray.shape
        camera_matrix = np.array([[width, 0, width / 2],
                                   [0, height, height / 2],
                                   [0, 0, 1]], dtype=np.float32)

        for i in range(len(ids)):
        # Get the corners of the detected marker
            marker_corners = corners[i][0]

        # Ensure marker_corners is a 2D array of type float32
            marker_corners = np.array(marker_corners, dtype=np.float32)

        # SolvePnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(object_points, marker_corners, camera_matrix, None)

            if success:
                # Print the pose estimation (translation and rotation vectors)
                print(f"Marker ID: {ids[i][0]} - Translation Vector (X, Y, Z): {tvec.ravel()} - Rotation Vector: {rvec.ravel()}")
                text_to_write = np.array2string(tvec.ravel(), separator=' ')
                file_path = 'coordinates.txt'

# Text to write as the first line
    

# Open the file in write mode
                file = open(file_path, 'w')
    # Write the text to the file
                file.write(text_to_write + '\n')  # Write the first line
# Close the file
                file.close() 
            

