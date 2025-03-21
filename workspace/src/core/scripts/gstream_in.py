import socket

class GStreamerVideoReceiver:
    def __init__(self):
        # Инициализация ROS ноды
        rospy.init_node('gstreamer_video_receiver', anonymous=True)
        
        # Создаем объект для преобразования изображения между ROS и OpenCV
        self.bridge = CvBridge()
        
        # Публикуем изображения в топик /nn_image
        self.image_pub = rospy.Publisher('/nn_image', Image, queue_size=10)
        
        # Проверка доступности порта перед запуском GStreamer
        if not self.is_port_open('127.0.0.1', 5001):
            rospy.logerr("Не удалось подключиться к видеопотоку на 127.0.0.1:5001. Попробуйте позже.")
            return

        # Запуск GStreamer для приема видеопотока
        self.gstreamer_command = (
            'gst-launch-1.0', 
            'udpsrc', 
            'address=127.0.0.1',  # IP адрес источника потока
            'port=5001', 
            'caps="application/x-rtp, media=video, payload=96"', 
            '!', 
            'rtph264depay', 
            '!', 
            'avdec_h264', 
            '!', 
            'videoconvert', 
            '!', 
            'appsink', 
            'sync=false'
        )

        # Запуск процесса GStreamer для захвата видео
        self.gstreamer_process = subprocess.Popen(self.gstreamer_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.cap = cv2.VideoCapture(self.gstreamer_process.stdout)

    def is_port_open(self, ip, port):
        """Проверка доступности порта с использованием socket."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1)
                s.connect((ip, port))
                return True
        except socket.error:
            return False

    def capture_and_publish(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                try:
                    # Преобразуем кадр в формат ROS Image
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    
                    # Публикуем изображение в топик /nn_image
                    self.image_pub.publish(ros_image)
                    
                except Exception as e:
                    rospy.logerr(f"Ошибка при преобразовании изображения: {e}")
            else:
                rospy.logwarn("Не удалось захватить кадр!")

if __name__ == '__main__':
    try:
        video_receiver = GStreamerVideoReceiver()
        if video_receiver:  # Только если видеопоток доступен
            video_receiver.capture_and_publish()
    except rospy.ROSInterruptException:
        pass
