""" Script that tries to access the camera stream of an ESP32-CAM board. """

import cv2

# Replace with your ESP32-CAM’s IP address. Remember to include the port number following the IP address.
stream_url = "http://192.168.1.185:81/stream"

print("Connecting to", stream_url)
cap = cv2.VideoCapture(stream_url)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("ESP32 Stream", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to quit
        break

cap.release()
cv2.destroyAllWindows()