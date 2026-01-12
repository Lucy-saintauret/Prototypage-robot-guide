#!/usr/bin/python

'''
This script allows the user to track a colored ball using a webcam. The user is prompted to place the ball in front of
the camera, and then draw a circle around the ball to select the region of interest. The script then calculates the HSV
range for the selected region and tracks the ball in real-time, providing feedback on the distance and direction to move
the robot to recenter the ball.
'''

import cv2
import numpy as np
from prototypage.publisher_node import BallPublisherNode
import rclpy

# Global variables for circle drawing
circle_center = None
circle_radius = 0
drawing_circle = False


def draw_circle(event, x, y, flags, param):
    global circle_center, circle_radius, drawing_circle, init_frame

    # When the user clicks the left mouse button, initialize the circle
    if event == cv2.EVENT_LBUTTONDOWN:
        circle_center = (x, y)
        circle_radius = 0
        drawing_circle = True

    # While the user is dragging the mouse, update the circle radius
    elif event == cv2.EVENT_MOUSEMOVE and drawing_circle:
        circle_radius = int(np.sqrt((x - circle_center[0]) ** 2 + (y - circle_center[1]) ** 2))
        temp_frame = init_frame.copy()
        cv2.circle(temp_frame, circle_center, circle_radius, (0, 255, 0), 2)
        cv2.imshow("BallTracker", temp_frame)

    # When the user releases the left mouse button, draw the circle
    elif event == cv2.EVENT_LBUTTONUP:
        drawing_circle = False
        circle_radius = int(np.sqrt((x - circle_center[0]) ** 2 + (y - circle_center[1]) ** 2))
        cv2.circle(init_frame, circle_center, circle_radius, (0, 255, 0), 2)
        cv2.imshow("BallTracker", init_frame)
        print("Circle drawn. Press any key to continue.")


def calculate_hsv_range():
    # Convert the selected region to HSV
    mask = np.zeros(init_frame.shape[:2], dtype=np.uint8)
    cv2.circle(mask, circle_center, circle_radius, 255, -1)
    hsv_frame = cv2.cvtColor(init_frame, cv2.COLOR_BGR2HSV)

    # Extract HSV values within the mask
    hsv_values = hsv_frame[mask == 255]

    # Compute 5th and 95th percentiles for more stable HSV ranges
    lower_hsv = np.percentile(hsv_values, 5, axis=0).astype(int)
    upper_hsv = np.percentile(hsv_values, 95, axis=0).astype(int)

    print(f"Lower HSV (5th percentile): {lower_hsv}")
    print(f"Upper HSV (95th percentile): {upper_hsv}")

    return lower_hsv, upper_hsv


def track_ball(lower_hsv, upper_hsv):
    # Reopen the camera for tracking
    cap = cv2.VideoCapture(0)
    print("Tracking the ball... Press 'q' to quit.")

    first_contour_diameter = None
    flag_first_iteration = True

    # Initialize the ROS publisher node
    rclpy.init()
    publisher_node = BallPublisherNode()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        position_diff = 0.0
        diameter_diff = 0.0

        # Convert the frame to HSV and apply the color range mask
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If any contours are detected
        if contours:
            # Find the largest contour, assuming it's the ball
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

            # Only proceed if the radius is above a certain threshold to avoid noise
            if radius > 10:
                # Draw the circle around the detected ball
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)  # Draw a center point

            # Calculate the diameter of the ball in pixels
            diameter = 2 * radius if radius > 0 else 0
            position = float(x)

            if flag_first_iteration:
                first_contour_diameter = diameter
                first_position = position
                flag_first_iteration = False

            # Calculate the difference in diameter between the first and current contours
            diameter_diff = float(-1 * (diameter - first_contour_diameter))

            # Calculate the difference in position between the first and current contours
            position_diff = float(position - first_position)

        # Publish the direction and distance to ROS topics
        publisher_node.publish_position(position_diff)
        publisher_node.publish_distance(diameter_diff)

        # Display the distance and direction to go to recenter the ball
        cv2.putText(frame, f"Distance difference: {diameter_diff:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Position difference: {position_diff:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display the tracking result
        frame = cv2.resize(frame, (window_size[1] // 2, window_size[0] // 2))
        cv2.imshow("BallTracker", frame)

        # Exit the tracking loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()



def main():
    # Open camera and prompt user to place the ball in view
    init_cap = cv2.VideoCapture(0)
    print("Place the ball in front of the camera, and press 'p' to take a photo.")

    # Display camera feed
    while True:
        _, init_frame = init_cap.read()
        # Get window size
        window_size = init_frame.shape[:2]
        # Resize the window
        init_frame = cv2.resize(init_frame, (window_size[1] // 2, window_size[0] // 2))

        cv2.imshow("BallTracker", init_frame)
        if cv2.waitKey(1) & 0xFF == ord('p'):
            break
    cv2.destroyAllWindows()

    # Allow the user to draw a circle to select the ball region
    cv2.imshow("BallTracker", init_frame)
    print("Draw a circle around the ball by clicking and dragging, starting from the center.")
    cv2.setMouseCallback("BallTracker", draw_circle)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Calculate the HSV range for the selected circular area
    if circle_center is not None and circle_radius > 0:
        min_hsv, max_hsv = calculate_hsv_range()
        init_cap.release()
        cv2.destroyAllWindows()

        track_ball(min_hsv, max_hsv)
    else:
        print("No valid circle was drawn.")


if __name__ == '__main__':
    main()