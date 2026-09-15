import cv2

# 尝试打开相机（设备索引 0,1,2...，通常是 0 或 2）
cap = cv2.VideoCapture(2)   # 根据你的系统调整，可以先用 0 尝试
if not cap.isOpened():
    print("无法打开相机，尝试索引 1")
    cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('Dabai Color', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
