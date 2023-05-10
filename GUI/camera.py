import cv2

cap = cv2.VideoCapture("rtsp://192.168.1.101:8554/test")

    
while True: 
    
    ret,img=cap.read()
    
    cv2.imshow('Video', img)
    
    if(cv2.waitKey(10) & 0xFF == ord('b')):
        break
