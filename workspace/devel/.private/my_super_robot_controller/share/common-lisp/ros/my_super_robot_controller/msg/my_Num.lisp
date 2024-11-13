; Auto-generated. Do not edit!


(cl:in-package my_super_robot_controller-msg)


;//! \htmlinclude my_Num.msg.html

(cl:defclass <my_Num> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0))
)

(cl:defclass my_Num (<my_Num>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <my_Num>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'my_Num)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_super_robot_controller-msg:<my_Num> is deprecated: use my_super_robot_controller-msg:my_Num instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <my_Num>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_super_robot_controller-msg:x-val is deprecated.  Use my_super_robot_controller-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <my_Num>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_super_robot_controller-msg:y-val is deprecated.  Use my_super_robot_controller-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <my_Num>) ostream)
  "Serializes a message object of type '<my_Num>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <my_Num>) istream)
  "Deserializes a message object of type '<my_Num>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<my_Num>)))
  "Returns string type for a message object of type '<my_Num>"
  "my_super_robot_controller/my_Num")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'my_Num)))
  "Returns string type for a message object of type 'my_Num"
  "my_super_robot_controller/my_Num")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<my_Num>)))
  "Returns md5sum for a message object of type '<my_Num>"
  "64be90712af6ea79ae6f103da824ffcf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'my_Num)))
  "Returns md5sum for a message object of type 'my_Num"
  "64be90712af6ea79ae6f103da824ffcf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<my_Num>)))
  "Returns full string definition for message of type '<my_Num>"
  (cl:format cl:nil "uint32 x~%uint32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'my_Num)))
  "Returns full string definition for message of type 'my_Num"
  (cl:format cl:nil "uint32 x~%uint32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <my_Num>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <my_Num>))
  "Converts a ROS message object to a list"
  (cl:list 'my_Num
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
