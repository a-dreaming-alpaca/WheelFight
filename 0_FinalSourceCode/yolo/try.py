from ultralytics import YOLO
import cv2

# 加载训练完自己训练出来的权重
model = YOLO("best.pt")

cap = cv2.VideoCapture(1)
conf_threshold = 0.15

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(
        frame,
        conf=conf_threshold,
        imgsz=640,
        verbose=False
    )
    res = results[0]

    has_debuff = False
    has_buff = False

    for box in res.boxes:
        cls = int(box.cls.item())
        conf = box.conf.item()
        x1,y1,x2,y2 = map(int,box.xyxy[0])

        if cls ==0:
            has_debuff=True
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),2)
            cv2.putText(frame,f"debuff {conf:.2f}",(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
        elif cls ==1:
            has_buff=True
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(frame,f"buff {conf:.2f}",(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)

    print(f"debuff:{has_debuff} | buff:{has_buff}")
    cv2.imshow("detect", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()