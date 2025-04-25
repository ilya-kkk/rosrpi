import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import cv2
import threading
import time

class GstSubscriber:
    def __init__(self, ip="127.0.0.1", port=5000):
        self.ip = ip
        self.port = port
        self.is_running = False
        self.pipeline = None
        self.appsink = None
        self.loop = None
        self.thread = None
        self.current_frame = None
        self.frame_lock = threading.Lock()

    def start(self):
        Gst.init(None)
        self.is_running = True
        
        # Создаем pipeline
        pipeline_str = (
            f"udpsrc address={self.ip} port={self.port} ! "
            "application/x-rtp,media=video,encoding-name=H264 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! "
            "videoconvert ! appsink name=sink"
        )
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        
        # Настраиваем appsink
        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("max-buffers", 1)
        self.appsink.set_property("drop", True)
        self.appsink.connect("new-sample", self._on_new_sample)
        
        # Запускаем pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Запускаем поток для обработки сообщений GStreamer
        self.thread = threading.Thread(target=self._run_loop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.is_running = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.thread:
            self.thread.join()

    def _run_loop(self):
        self.loop = GLib.MainLoop()
        self.loop.run()

    def _on_new_sample(self, appsink):
        sample = appsink.pull_sample()
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value("width")
            height = structure.get_value("height")
            
            # Получаем данные из буфера
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if success:
                # Преобразуем данные в numpy массив
                frame = np.ndarray(
                    shape=(height, width, 3),
                    dtype=np.uint8,
                    buffer=map_info.data
                )
                
                # Копируем кадр
                with self.frame_lock:
                    self.current_frame = frame.copy()
                
                buffer.unmap(map_info)
                return Gst.FlowReturn.OK
        return Gst.FlowReturn.ERROR

    def get_frame(self):
        with self.frame_lock:
            if self.current_frame is not None:
                return self.current_frame.copy()
        return None

class GstPublisher:
    def __init__(self, ip="127.0.0.1", port=5001):
        self.ip = ip
        self.port = port
        self.is_running = False
        self.pipeline = None
        self.appsrc = None
        self.loop = None
        self.thread = None

    def start(self):
        Gst.init(None)
        self.is_running = True
        
        # Создаем pipeline
        pipeline_str = (
            "appsrc name=src ! videoconvert ! "
            "x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! "
            "rtph264pay ! udpsink host={} port={}"
        ).format(self.ip, self.port)
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("src")
        
        # Настраиваем appsrc
        self.appsrc.set_property("format", Gst.Format.TIME)
        self.appsrc.set_property("is-live", True)
        
        # Запускаем pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Запускаем поток для обработки сообщений GStreamer
        self.thread = threading.Thread(target=self._run_loop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.is_running = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.thread:
            self.thread.join()

    def _run_loop(self):
        self.loop = GLib.MainLoop()
        self.loop.run()

    def publish_frame(self, frame):
        if not self.is_running or frame is None:
            return False
            
        try:
            # Преобразуем кадр в формат GStreamer
            height, width = frame.shape[:2]
            buffer = Gst.Buffer.new_wrapped(frame.tobytes())
            
            # Устанавливаем caps
            caps = Gst.Caps.new_empty_simple("video/x-raw")
            caps.set_value("format", "RGB")
            caps.set_value("width", width)
            caps.set_value("height", height)
            
            # Отправляем кадр
            self.appsrc.set_caps(caps)
            self.appsrc.emit("push-buffer", buffer)
            return True
            
        except Exception as e:
            print(f"Error publishing frame: {e}")
            return False 