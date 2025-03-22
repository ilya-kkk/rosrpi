#!/usr/bin/env python3
import cv2
import os
import time
import subprocess
from ultralytics import YOLO

# Загрузка модели YOLO (если модели нет в указанном пути, она будет скачана)
model_path = "/nn_ws/yolo8n.pt"
if not os.path.exists(model_path):
    print("Модель не найдена! Скачивание...")
    model = YOLO('yolo8n.pt')
else:
    model = YOLO(model_path)

# Классы, которые нас интересуют (например, человек, автомобиль, и т.д.)
target_classes = [0, 2, 14, 15]

def filter_detections(results, target_classes):
    """Фильтрация обнаруженных объектов по целевым классам."""
    filtered = []
    boxes = results[0].boxes
    for box in boxes:
        if int(box.cls.item()) in target_classes:
            filtered.append(box)
    return filtered

# Конвейер для приема видеопотока по UDP (порт 5000)
input_pipeline = (
    "udpsrc port=5000 ! "
    "application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false"
)

# Конвейер для отправки обработанного видео (на порт 5001)
output_pipeline = (
    "appsrc name=mysrc caps=video/x-raw,format=BGR,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! "
    "x264enc speed-preset=ultrafast tune=zerolatency ! "
    "rtph264pay config-interval=1 pt=96 ! "
    "udpsink host=127.0.0.1 port=5001"
)

def is_gstreamer_running():
    """Проверяет, запущен ли процесс GStreamer (gst-launch-1.0)."""
    try:
        result = subprocess.run(["pgrep", "gst-launch-1.0"], capture_output=True, text=True)
        return result.returncode == 0
    except Exception as e:
        print(f"Ошибка при проверке GStreamer: {e}")
        return False

def open_video_stream():
    """
    Попытка подключения к видеопотоку с проверкой работы GStreamer.
    Если поток не доступен, повторяет попытку раз в секунду.
    """
    while True:
        if not is_gstreamer_running():
            print("GStreamer не запущен. Ожидание...")
            time.sleep(1)
            continue

        cap = cv2.VideoCapture(input_pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            print("Видеопоток успешно подключен")
            return cap
        print("Ошибка подключения. Повторная попытка через 1 секунду...")
        time.sleep(1)

def main():
    # Подключаемся к входному видеопотоку с повторными попытками
    cap = open_video_stream()
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30

    # Инициализируем VideoWriter для отправки обработанного видео
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

        # Применяем модель YOLO для обнаружения объектов
        results = model(frame)
        filtered_boxes = filter_detections(results, target_classes)

        # Рисуем рамки и подписи для обнаруженных объектов
        for box in filtered_boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = box.conf.item()
            cls = int(box.cls.item())
            label = f"{model.names[cls]} {conf:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Отправляем обработанный кадр через выходной GStreamer pipeline
        out.write(frame)

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
