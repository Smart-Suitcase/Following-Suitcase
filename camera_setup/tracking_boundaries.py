import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Start a loop to continuously capture frames from the webcam
while True:
    # Capture a frame
    ret, frame = cap.read()
    
    # Display the frame
    cv2.imshow('frame', frame)
    
    # Wait for a key press and check if it's the escape key
    key = cv2.waitKey(1)
    if key == 27:
        break
    
    # If the spacebar is pressed, select a region of interest (ROI) in the frame
    elif key == 32:
        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Select the ROI using the mouse
        roi = cv2.selectROI('frame', frame, fromCenter=False, showCrosshair=True)
        
        # Get the average HSV value of the ROI
        roi_hsv = cv2.cvtColor(frame[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]], cv2.COLOR_BGR2HSV)
        h, s, v = cv2.mean(roi_hsv)[:3]
        print("Selected region average HSV: ({:.0f}, {:.0f}, {:.0f})".format(h, s, v))
        
        # Calculate the lower and upper bounds for tracking
        lower_bound = np.array([h-15, s-70, v-70])
        upper_bound = np.array([h+15, s+70, v+70])

        print("Proposed lower bound: {}".format(lower_bound))
        print("Proposed upper bound: {}".format(upper_bound))
        
        # Create a mask using the lower and upper bounds
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Find the contours of the object in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If any contours are found, draw a bounding box around the largest one
        if len(contours) > 0:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Get the bounding box coordinates of the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            # Draw the bounding box around the object in the original frame
           
