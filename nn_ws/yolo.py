#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import os
import time
import socket
import torch
from ultralytics import YOLO
import subprocess

# Проверка доступности CUDA
print(" \033[1;33m Проверка доступности CUDA \033[0m")
if torch.cuda.is_available():
    device = torch.device('cuda')
    print(f" \033[1;32m CUDA доступна! Используется GPU: {torch.cuda.get_device_name(0)} \033[0m")
else:
    device = torch.device('cpu')
    print(" \033[1;31m CUDA недоступна! Используется CPU \033[0m")

# Проверка доступности GStreamer
print("\n\033[1;33m Проверка GStreamer \033[0m")
try:
    # Проверяем наличие необходимых плагинов
    plugins = ['udpsrc', 'rtpjpegdepay', 'jpegdec', 'videoconvert', 'appsink', 'appsrc', 'jpegenc', 'rtpjpegpay']
    for plugin in plugins:
        result = subprocess.run(['gst-inspect-1.0', plugin], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"\033[1;32m Плагин {plugin} доступен \033[0m")
        else:
            print(f"\033[1;31m Плагин {plugin} НЕ доступен \033[0m")
except Exception as e:
    print(f"\033[1;31m Ошибка при проверке GStreamer: {e} \033[0m")

# Загрузка модели YOLO
print("\n\033[1;33m NN init start \033[0m")
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

# GStreamer pipelines для работы с кадрами
input_pipeline = 'udpsrc port=5000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)JPEG, payload=(int)26" ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink'
output_pipeline = 'appsrc ! videoconvert ! jpegenc ! rtpjpegpay ! udpsink host=127.0.0.1 port=5001'

def main():
    print("\n\033[1;33m Инициализация GStreamer... \033[0m")
    print(f"Входной pipeline: {input_pipeline}")
    print(f"Выходной pipeline: {output_pipeline}")
    
    # Инициализация входного потока
    cap = cv2.VideoCapture(input_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("\033[1;31m Ошибка: Не удалось открыть входной поток GStreamer \033[0m")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('0.0.0.0', 5000))
            print("\033[1;33m Ожидание данных на порту 5000... \033[0m")
            data, addr = sock.recvfrom(1024)
            print(f"\033[1;32m Получены данные от {addr} \033[0m")
            sock.close()
        except Exception as e:
            print(f"\033[1;31m Ошибка при проверке порта: {e} \033[0m")
        return

    # Получаем размеры кадра
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Инициализация выходного потока с частотой 1 кадр в секунду
    out = cv2.VideoWriter(output_pipeline, cv2.CAP_GSTREAMER, 0, 1, (width, height), True)
    if not out.isOpened():
        print("\033[1;31m Ошибка: Не удалось открыть выходной поток GStreamer \033[0m")
        return

    print("\033[1;32m GStreamer успешно инициализирован \033[0m")
    print("Начало обработки кадров...")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("\033[0;31m Ошибка чтения кадра \033[0m")
            continue

        try:
            # Обработка кадра YOLO
            results = model(frame)
            # filtered_boxes = filter_detections(results, target_classes)
            filtered_boxes = results
            
            # Рисуем рамки и подписи
            for box in filtered_boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = box.conf.item()
                cls = int(box.cls.item())
                label = f"{model.names[cls]} {conf:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Отправка обработанного кадра
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
