#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
import time
import socket
import torch
from ultralytics import YOLO

# Проверка доступности CUDA
print(" \033[1;33m Проверка доступности CUDA \033[0m")
if torch.cuda.is_available():
    device = torch.device('cuda')
    print(f" \033[1;32m CUDA доступна! Используется GPU: {torch.cuda.get_device_name(0)} \033[0m")
else:
    device = torch.device('cpu')
    print(" \033[1;31m CUDA недоступна! Используется CPU \033[0m")

# Загрузка модели YOLO
print(" \033[1;33m NN init start \033[0m")
model_path = "yolo8n.pt"
if not os.path.exists(model_path):
    print("Модель не найдена! Скачивание...")
    model = YOLO('yolo8n.pt')
else:
    model = YOLO(model_path)

# Перемещение модели на GPU
model.to(device)

# Целевые классы для обнаружения
target_classes = [0, 2, 14, 15]  # person, car, bird, cat

def filter_detections(results, target_classes):
    filtered = []
    boxes = results[0].boxes
    for box in boxes:
        if int(box.cls.item()) in target_classes:
            filtered.append(box)
    return filtered

# GStreamer pipelines
input_pipeline = 'udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink'
output_pipeline = 'appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port=5001'

def main():
    print("Инициализация GStreamer...")
    cap = cv2.VideoCapture(input_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Ошибка: Не удалось открыть входной поток GStreamer")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30

    out = cv2.VideoWriter(output_pipeline, cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
    if not out.isOpened():
        print("Ошибка: Не удалось открыть выходной поток GStreamer")
        return

    print("Начало обработки видеопотока...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("\033[0;31m Ошибка чтения кадра \033[0m")
            continue

        try:
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
            print("\033[0;32m Кадр обработан и отправлен \033[0m")

        except Exception as e:
            print(f"\033[0;31m Ошибка при обработке кадра: {str(e)} \033[0m")
            continue

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
