import cv2
import torch
import time
from ultralytics import YOLO

# Загрузка модели YOLO11n
model = YOLO('yolo11n.pt')

# Список интересующих классов (ID для COCO)
target_classes = [0, 2, 14, 15]

def filter_detections(results, target_classes):
    """Фильтрует обнаружения, оставляя только интересующие классы."""
    filtered = []
    boxes = results[0].boxes
    for box in boxes:
        if int(box.cls.item()) in target_classes:
            filtered.append(box)
    return filtered

# GStreamer пайплайны
input_pipeline = (
    "udpsrc port=5000 ! "
    "application/x-rtp, encoding-name=H264 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)
output_pipeline = (
    "appsrc ! videoconvert ! "
    "x264enc speed-preset=ultrafast tune=zerolatency ! "
    "rtph264pay ! udpsink host=192.168.0.108 port=5001"
)

def open_video_stream():
    """Попытка подключения к видеопотоку с повторными попытками каждую секунду."""
    while True:
        cap = cv2.VideoCapture(input_pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            print("Видеопоток успешно подключен")
            return cap
        print("Ошибка подключения. Повторная попытка через 1 секунду...")
        time.sleep(1)

def main():
    cap = open_video_stream()

    # Получение параметров видеопотока
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30

    out = cv2.VideoWriter(output_pipeline, cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
    if not out.isOpened():
        print("Ошибка открытия выходного видеопотока")
        cap.release()
        return

    print("Начало обработки видеопотока...")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Поток потерян. Переподключение...")
            cap.release()
            cap = open_video_stream()
            continue

        results = model(frame)
        filtered_boxes = filter_detections(results, target_classes)

        for box in filtered_boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = box.conf.item()
            cls = int(box.cls.item())
            label = f"{model.names[cls]} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        out.write(frame)

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
