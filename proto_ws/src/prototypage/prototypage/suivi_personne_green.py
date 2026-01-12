#!/usr/bin/env python

import cv2
import numpy as np
import rclpy
from prototypage.publisher_node import BallPublisherNode

# For OpenCV2 image display
WINDOW_NAME = 'suivi_personne_green'

# Known parameters
KNOWN_DIAMETER = 3.0  # Taille réelle de la balle en cm
FOCAL_LENGTH = 1080  # Longueur focale en pixels (à ajuster selon votre caméra)


def track():
    # Initialize the ROS publisher node
    rclpy.init()
    publisher_node = BallPublisherNode()

    cap = cv2.VideoCapture("/dev/video0")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Blur the image to reduce noise
        blur = cv2.GaussianBlur(frame, (5, 5), 0)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image for only green colors
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 200, 200])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Blur the mask
        bmask = cv2.GaussianBlur(mask, (5, 5), 0)

        position = 0.0
        distance_to_10cm = 0.0
        radius = 0
        x, y = 0, 0

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calculate relative position and distance if the centroid exists
        if contours:
            # Find the largest contour, assuming it's the ball
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

            # Only proceed if the radius is above a certain threshold to avoid noise
            if radius > 10:
                # Draw the circle around the detected ball
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)  # Draw a center point

            # Find the radius of the ball by contour detection
            contours, _ = cv2.findContours(bmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                if radius > 0:
                    # Put black circle in the center of the ball
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 0), -1)

                # Determine image center
                image_center_x = frame.shape[1] // 2

                # Calculate relative position from the image center
                centroid_x = int(x)
                position = float(centroid_x - image_center_x)

                # Calculate the distance
                real_distance = (KNOWN_DIAMETER * FOCAL_LENGTH) / (2 * radius)
                distance_to_10cm = float(real_distance - 10.0)

        # Publish the direction and distance to ROS topics
        publisher_node.publish_position(position)
        publisher_node.publish_distance(distance_to_10cm)

        # Draw circle and display distance to 30 cm and position to center on the image
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"Distance: {distance_to_10cm:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)
        cv2.putText(frame, f"Position difference: {position:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)



def main():
    track()


if __name__ == '__main__':
    main()
