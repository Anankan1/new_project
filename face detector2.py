import cv2
from mtcnn import MTCNN

def moving_direction(x,y,x_previous,y_previous):
    if x>x_previous:
        if y == y_previous:
            print("image moving left to right")
        elif y> y_previous:
            print("image moving to right bottom corner")
        else:
            print ("image moving to right top corner")
    elif x==x_previous:
        if y == y_previous:
            print("image doesnot move")
        elif y> y_previous:
            print("image moving to straight bottom")
        else:
            print ("image moving to upward")
    else:
        if y == y_previous:
            print("image moving right to left")
        elif y> y_previous:
            print("image moving to left bottom corner")
        else:
            print ("image moving to left top corner")
# Initialize the MTCNN detector
detector = MTCNN()

# Initialize the video capture
cap = cv2.VideoCapture(0)  # Change the argument to your video file path if not using a webcam
list1=[]
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if ret:
        # Convert frame to RGB (required by MTCNN)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detect faces in the frame
        faces = detector.detect_faces(rgb_frame)
        if faces:
            face = faces[0]
            x, y, w, h = face['box']
            a = x + w / 2
            b = y + h / 2
            list1.append([a,b])
            print(list1)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        if len(list1)>2:
            # for i in range(len(list1)):
            moving_direction(list1[-1][0],list1[-1][1],list1[-2][0],list1[-2][1])
        # Draw rectangles around the faces
        # for face in faces:
        #     x, y, w, h = face['box']
            
        #     cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Face Detection', frame)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture
cap.release()
cv2.destroyAllWindows()



