#!/usr/bin/env python3
import cv2
import torch
from ultralytics import YOLO

# Загрузка модели YOLO11n
model = YOLO('yolo11n.pt')

# Список интересующих классов (ID для COCO)
# Здесь предполагается, что модель обучена на COCO:
# 0: person, 2: car, 14: bench, 15: potted plant (можно заменить на "tree", если имеется соответствующий класс)
target_classes = [0, 2, 14, 15]

def filter_detections(results, target_classes):
    """
    Фильтрует обнаружения, оставляя только те, у которых класс входит в target_classes.
    Предполагается, что results — список объектов, возвращаемых моделью YOLO.
    """
    filtered = []
    # В новых моделях Ultralytics результат обычно имеет атрибут boxes
    boxes = results[0].boxes  # boxes - это объект Boxes с полями: .xyxy, .conf, .cls
    for box in boxes:
        cls = int(box.cls.item())
        if cls in target_classes:
            filtered.append(box)
    return filtered

# GStreamer-пайплайн для приема видеопотока по UDP (на порту 5000)
input_pipeline = (
    "udpsrc port=5000 ! "
    "application/x-rtp, encoding-name=H264 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)

# GStreamer-пайплайн для отправки видеопотока по UDP (на порт 5001, адрес можно изменить)
output_pipeline = (
    "appsrc ! videoconvert ! "
    "x264enc speed-preset=ultrafast tune=zerolatency ! "
    "rtph264pay ! udpsink host=192.168.0.108 port=5001"
)

# Открытие входного видеопотока через GStreamer
cap = cv2.VideoCapture(input_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Ошибка открытия входного видеопотока")
    exit()

# Определение параметров видеопотока
width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps    = cap.get(cv2.CAP_PROP_FPS)
if fps == 0:
    fps = 30  # значение по умолчанию

# Открытие выходного видеопотока через GStreamer
out = cv2.VideoWriter(output_pipeline, cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
if not out.isOpened():
    print("Ошибка открытия выходного видеопотока")
    cap.release()
    exit()

print("Начало обработки видеопотока...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Получаем детекции от модели (результаты возвращаются в виде списка объектов)
    results = model(frame)
    # Фильтруем обнаружения по интересующим нас классам
    filtered_boxes = filter_detections(results, target_classes)

    # Отрисовка боксов на кадре
    for box in filtered_boxes:
        # Получаем координаты бокса (x1, y1, x2, y2)
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf = box.conf.item()
        cls = int(box.cls.item())
        label = f"{model.names[cls]} {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Запись обработанного кадра в выходной поток
    out.write(frame)

    # (Опционально) Отображение результата локально
    # cv2.imshow("YOLO11n Detection", frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# Освобождение ресурсов
cap.release()
out.release()
cv2.destroyAllWindows()
