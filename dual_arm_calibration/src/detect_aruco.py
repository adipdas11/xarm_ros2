# aruco_detection.py
# Enhanced Python script to detect ArUco markers from webcam (camera 0) using OpenCV
# Improved detection via parameter tuning and image preprocessing

import cv2
import cv2.aruco as aruco
import sys

# ArUco dictionary
ARUCO_DICT = aruco.DICT_4X4_100


def detect_markers(camera_id=0):
    """
    Opens the video stream from camera 0 and detects 4x4 ArUco markers in real-time.
    Press 'q' to quit.
    """
    # Initialize video capture
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Error: Unable to open camera {camera_id}")
        sys.exit(1)

    # Load dictionary
    try:
        aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
    except AttributeError:
        aruco_dict = aruco.Dictionary_get(ARUCO_DICT)

    # Create and tune detector parameters
    try:
        params = aruco.DetectorParameters_create()
    except AttributeError:
        params = aruco.DetectorParameters()
    # Adaptive threshold settings
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 7
    # Corner refinement for subpixel accuracy
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30

    print("Starting camera stream with enhanced detection. Press 'q' to exit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame")
            break

        # Preprocess: convert to grayscale and apply CLAHE for contrast improvement
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray_eq = clahe.apply(gray)

        # Detect markers
        corners, ids, rejected = aruco.detectMarkers(gray_eq, aruco_dict, parameters=params)

        # Optionally draw rejected candidates
        # aruco.drawDetectedMarkers(frame, rejected, borderColor=(0,0,255))

        if ids is not None and len(ids) > 0:
            # Draw detected marker borders and IDs
            aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"Detected marker IDs: {ids.flatten()}")

        # Display the result
        cv2.imshow('Enhanced ArUco Detection', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    detect_markers()