; Auto-generated. Do not edit!


(cl:in-package lightrover_ros-srv)


;//! \htmlinclude Wrc201Msg-request.msg.html

(cl:defclass <Wrc201Msg-request> (roslisp-msg-protocol:ros-message)
  ((addr
    :reader addr
    :initarg :addr
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0)
   (length
    :reader length
    :initarg :length
    :type cl:fixnum
    :initform 0)
   (cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform ""))
)

(cl:defclass Wrc201Msg-request (<Wrc201Msg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Wrc201Msg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Wrc201Msg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lightrover_ros-srv:<Wrc201Msg-request> is deprecated: use lightrover_ros-srv:Wrc201Msg-request instead.")))

(cl:ensure-generic-function 'addr-val :lambda-list '(m))
(cl:defmethod addr-val ((m <Wrc201Msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lightrover_ros-srv:addr-val is deprecated.  Use lightrover_ros-srv:addr instead.")
  (addr m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Wrc201Msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lightrover_ros-srv:data-val is deprecated.  Use lightrover_ros-srv:data instead.")
  (data m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <Wrc201Msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lightrover_ros-srv:length-val is deprecated.  Use lightrover_ros-srv:length instead.")
  (length m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <Wrc201Msg-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lightrover_ros-srv:cmd-val is deprecated.  Use lightrover_ros-srv:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Wrc201Msg-request>) ostream)
  "Serializes a message object of type '<Wrc201Msg-request>"
  (cl:let* ((signed (cl:slot-value msg 'addr)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'length)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Wrc201Msg-request>) istream)
  "Deserializes a message object of type '<Wrc201Msg-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'addr) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'length) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Wrc201Msg-request>)))
  "Returns string type for a service object of type '<Wrc201Msg-request>"
  "lightrover_ros/Wrc201MsgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Wrc201Msg-request)))
  "Returns string type for a service object of type 'Wrc201Msg-request"
  "lightrover_ros/Wrc201MsgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Wrc201Msg-request>)))
  "Returns md5sum for a message object of type '<Wrc201Msg-request>"
  "43791c45179089218511643bd58fdb58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Wrc201Msg-request)))
  "Returns md5sum for a message object of type 'Wrc201Msg-request"
  "43791c45179089218511643bd58fdb58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Wrc201Msg-request>)))
  "Returns full string definition for message of type '<Wrc201Msg-request>"
  (cl:format cl:nil "int8 addr~%int64 data~%int8 length~%string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Wrc201Msg-request)))
  "Returns full string definition for message of type 'Wrc201Msg-request"
  (cl:format cl:nil "int8 addr~%int64 data~%int8 length~%string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Wrc201Msg-request>))
  (cl:+ 0
     1
     8
     1
     4 (cl:length (cl:slot-value msg 'cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Wrc201Msg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Wrc201Msg-request
    (cl:cons ':addr (addr msg))
    (cl:cons ':data (data msg))
    (cl:cons ':length (length msg))
    (cl:cons ':cmd (cmd msg))
))
;//! \htmlinclude Wrc201Msg-response.msg.html

(cl:defclass <Wrc201Msg-response> (roslisp-msg-protocol:ros-message)
  ((readData
    :reader readData
    :initarg :readData
    :type cl:integer
    :initform 0))
)

(cl:defclass Wrc201Msg-response (<Wrc201Msg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Wrc201Msg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Wrc201Msg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lightrover_ros-srv:<Wrc201Msg-response> is deprecated: use lightrover_ros-srv:Wrc201Msg-response instead.")))

(cl:ensure-generic-function 'readData-val :lambda-list '(m))
(cl:defmethod readData-val ((m <Wrc201Msg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lightrover_ros-srv:readData-val is deprecated.  Use lightrover_ros-srv:readData instead.")
  (readData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Wrc201Msg-response>) ostream)
  "Serializes a message object of type '<Wrc201Msg-response>"
  (cl:let* ((signed (cl:slot-value msg 'readData)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Wrc201Msg-response>) istream)
  "Deserializes a message object of type '<Wrc201Msg-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'readData) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Wrc201Msg-response>)))
  "Returns string type for a service object of type '<Wrc201Msg-response>"
  "lightrover_ros/Wrc201MsgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Wrc201Msg-response)))
  "Returns string type for a service object of type 'Wrc201Msg-response"
  "lightrover_ros/Wrc201MsgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Wrc201Msg-response>)))
  "Returns md5sum for a message object of type '<Wrc201Msg-response>"
  "43791c45179089218511643bd58fdb58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Wrc201Msg-response)))
  "Returns md5sum for a message object of type 'Wrc201Msg-response"
  "43791c45179089218511643bd58fdb58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Wrc201Msg-response>)))
  "Returns full string definition for message of type '<Wrc201Msg-response>"
  (cl:format cl:nil "int64 readData~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Wrc201Msg-response)))
  "Returns full string definition for message of type 'Wrc201Msg-response"
  (cl:format cl:nil "int64 readData~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Wrc201Msg-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Wrc201Msg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Wrc201Msg-response
    (cl:cons ':readData (readData msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Wrc201Msg)))
  'Wrc201Msg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Wrc201Msg)))
  'Wrc201Msg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Wrc201Msg)))
  "Returns string type for a service object of type '<Wrc201Msg>"
  "lightrover_ros/Wrc201Msg")