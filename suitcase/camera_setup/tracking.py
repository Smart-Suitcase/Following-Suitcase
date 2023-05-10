import cv2
import numpy as np

# Define the lower and upper boundaries of the object's color in the HSV color space
# You can adjust these values depending on the color of the object you want to track
# Proposed lower bound: [154.18413978  38.7188172   36.36586022]
# Proposed upper bound: [174.18413978 138.7188172  136.36586022]
lower_color = np.array([2.6, 70.4, 104.5])
upper_color = np.array([32.6, 210.4, 244.5])

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Start a loop to continuously capture frames from the webcam
while True:
    # Capture a frame
    ret, frame = cap.read()
    
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create a mask using the lower and upper boundaries of the object's color
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    # Find the contours of the object in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # If any contours are found, draw a bounding box around the largest one
    if len(contours) > 0:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        # Get the bounding box coordinates of the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)
        # Draw the bounding box around the object in the original frame
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    # Display the original frame and the mask
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    
    # Wait for a key press and check if it's the escape key
    key = cv2.waitKey(1)
    if key == 27:
        break

# Release the webcam and close the windows
cap.release()
cv2.destroyAllWindows()
