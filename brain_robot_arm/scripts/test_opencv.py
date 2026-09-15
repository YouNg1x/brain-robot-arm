import cv2

# 尝试不同的设备索引（0,1,2...）
cap = cv2.VideoCapture(0)   # 0 通常是第一个摄像头

if not cap.isOpened():
    print("无法打开摄像头，尝试索引 1")
    cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('Dabai Color', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

