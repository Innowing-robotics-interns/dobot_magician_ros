import cv2

def show_camera_feed():
    # Initialize the camera capture object with the cv2.VideoCapture class.
    # The parameter '0' indicates the first camera device on your computer.
    cap = cv2.VideoCapture(4)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    try:
        # Loop to continuously get frames and display them.
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            # If frame is read correctly, ret is True.
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # Display the resulting frame in a window
            cv2.imshow('Camera Feed', frame)

            # Press 'q' on the keyboard to exit the loop
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        # When everything is done, release the capture
        cap.release()
        cv2.destroyAllWindows()

# Run the function to show the camera feed.
show_camera_feed()