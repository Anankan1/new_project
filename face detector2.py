import cv2
from mtcnn import MTCNN

# Initialize the MTCNN detector
detector = MTCNN()

# Initialize the video capture
cap = cv2.VideoCapture(0)  # Change the argument to your video file path if not using a webcam

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if ret:
        # Convert frame to RGB (required by MTCNN)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detect faces in the frame
        faces = detector.detect_faces(rgb_frame)

        # Draw rectangles around the faces
        for face in faces:
            x, y, w, h = face['box']
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Face Detection', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()
