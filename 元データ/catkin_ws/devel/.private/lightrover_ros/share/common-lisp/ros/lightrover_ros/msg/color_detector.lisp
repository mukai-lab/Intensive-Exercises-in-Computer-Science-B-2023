; Auto-generated. Do not edit!


(cl:in-package lightrover_ros-msg)


;//! \htmlinclude color_detector.msg.html

(cl:defclass <color_detector> (roslisp-msg-protocol:ros-message)
  ((color_type
    :reader color_type
    :initarg :color_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass color_detector (<color_detector>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <color_detector>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'color_detector)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lightrover_ros-msg:<color_detector> is deprecated: use lightrover_ros-msg:color_detector instead.")))

(cl:ensure-generic-function 'color_type-val :lambda-list '(m))
(cl:defmethod color_type-val ((m <color_detector>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lightrover_ros-msg:color_type-val is deprecated.  Use lightrover_ros-msg:color_type instead.")
  (color_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <color_detector>) ostream)
  "Serializes a message object of type '<color_detector>"
  (cl:let* ((signed (cl:slot-value msg 'color_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <color_detector>) istream)
  "Deserializes a message object of type '<color_detector>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color_type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<color_detector>)))
  "Returns string type for a message object of type '<color_detector>"
  "lightrover_ros/color_detector")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'color_detector)))
  "Returns string type for a message object of type 'color_detector"
  "lightrover_ros/color_detector")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<color_detector>)))
  "Returns md5sum for a message object of type '<color_detector>"
  "28ff8e773f1e211eaff66787abb1695e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'color_detector)))
  "Returns md5sum for a message object of type 'color_detector"
  "28ff8e773f1e211eaff66787abb1695e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<color_detector>)))
  "Returns full string definition for message of type '<color_detector>"
  (cl:format cl:nil "int8 color_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'color_detector)))
  "Returns full string definition for message of type 'color_detector"
  (cl:format cl:nil "int8 color_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <color_detector>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <color_detector>))
  "Converts a ROS message object to a list"
  (cl:list 'color_detector
    (cl:cons ':color_type (color_type msg))
))
