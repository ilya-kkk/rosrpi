;; Auto-generated. Do not edit!


(when (boundp 'my_super_robot_controller::my_Num)
  (if (not (find-package "MY_SUPER_ROBOT_CONTROLLER"))
    (make-package "MY_SUPER_ROBOT_CONTROLLER"))
  (shadow 'my_Num (find-package "MY_SUPER_ROBOT_CONTROLLER")))
(unless (find-package "MY_SUPER_ROBOT_CONTROLLER::MY_NUM")
  (make-package "MY_SUPER_ROBOT_CONTROLLER::MY_NUM"))

(in-package "ROS")
;;//! \htmlinclude my_Num.msg.html


(defclass my_super_robot_controller::my_Num
  :super ros::object
  :slots (_x _y ))

(defmethod my_super_robot_controller::my_Num
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; uint32 _x
    4
    ;; uint32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _x
       (write-long _x s)
     ;; uint32 _y
       (write-long _y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_super_robot_controller::my_Num :md5sum-) "64be90712af6ea79ae6f103da824ffcf")
(setf (get my_super_robot_controller::my_Num :datatype-) "my_super_robot_controller/my_Num")
(setf (get my_super_robot_controller::my_Num :definition-)
      "uint32 x
uint32 y
")



(provide :my_super_robot_controller/my_Num "64be90712af6ea79ae6f103da824ffcf")


