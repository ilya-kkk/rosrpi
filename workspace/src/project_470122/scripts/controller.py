#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


class ControllerCommandPublisher:
    def __init__(self):
        self.flag = False
        self.value = 0.0
        self.speed = 1.8 

        # Инициализация публикователя и подписчиков
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.flag_subscriber = rospy.Subscriber("/flag", Bool, self.flag_cb)
        self.value_subscriber = rospy.Subscriber("/value", Float64, self.value_cb)

        self.rate = rospy.Rate(10)  # Частота публикации 10 Гц

    def flag_cb(self, msg):
        """Обратный вызов для флага."""
        self.flag = msg.data

    def value_cb(self, msg):
        """Обратный вызов для значения угловой скорости."""
        self.value = msg.data

    def publish_commands(self):
        """Публикация команд движения."""
        while not rospy.is_shutdown():
            if self.flag:  # Публикуем команды только если флаг установлен
                vel = self.value / 5.0  # Вычисление угловой скорости
                output_msg = Twist()
                output_msg.linear.x = self.speed
                output_msg.angular.z = -vel

                self.cmd_publisher.publish(output_msg)
            else:
                # Останавливаем робот, если флаг сброшен
                output_msg = Twist()
                self.cmd_publisher.publish(output_msg)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("controller_command_publisher")
    controller = ControllerCommandPublisher()

    try:
        controller.publish_commands()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller Command Publisher node terminated.")
