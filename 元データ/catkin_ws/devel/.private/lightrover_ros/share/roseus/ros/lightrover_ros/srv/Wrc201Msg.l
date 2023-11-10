;; Auto-generated. Do not edit!


(when (boundp 'lightrover_ros::Wrc201Msg)
  (if (not (find-package "LIGHTROVER_ROS"))
    (make-package "LIGHTROVER_ROS"))
  (shadow 'Wrc201Msg (find-package "LIGHTROVER_ROS")))
(unless (find-package "LIGHTROVER_ROS::WRC201MSG")
  (make-package "LIGHTROVER_ROS::WRC201MSG"))
(unless (find-package "LIGHTROVER_ROS::WRC201MSGREQUEST")
  (make-package "LIGHTROVER_ROS::WRC201MSGREQUEST"))
(unless (find-package "LIGHTROVER_ROS::WRC201MSGRESPONSE")
  (make-package "LIGHTROVER_ROS::WRC201MSGRESPONSE"))

(in-package "ROS")





(defclass lightrover_ros::Wrc201MsgRequest
  :super ros::object
  :slots (_addr _data _length _cmd ))

(defmethod lightrover_ros::Wrc201MsgRequest
  (:init
   (&key
    ((:addr __addr) 0)
    ((:data __data) 0)
    ((:length __length) 0)
    ((:cmd __cmd) "")
    )
   (send-super :init)
   (setq _addr (round __addr))
   (setq _data (round __data))
   (setq _length (round __length))
   (setq _cmd (string __cmd))
   self)
  (:addr
   (&optional __addr)
   (if __addr (setq _addr __addr)) _addr)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:serialization-length
   ()
   (+
    ;; int8 _addr
    1
    ;; int64 _data
    8
    ;; int8 _length
    1
    ;; string _cmd
    4 (length _cmd)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _addr
       (write-byte _addr s)
     ;; int64 _data
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _data (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _data) (= (length (_data . bv)) 2)) ;; bignum
              (write-long (ash (elt (_data . bv) 0) 0) s)
              (write-long (ash (elt (_data . bv) 1) -1) s))
             ((and (class _data) (= (length (_data . bv)) 1)) ;; big1
              (write-long (elt (_data . bv) 0) s)
              (write-long (if (>= _data 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _data s)(write-long (if (>= _data 0) 0 #xffffffff) s)))
     ;; int8 _length
       (write-byte _length s)
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _addr
     (setq _addr (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _addr 127) (setq _addr (- _addr 256)))
   ;; int64 _data
#+(or :alpha :irix6 :x86_64)
      (setf _data (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _data (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int8 _length
     (setq _length (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _length 127) (setq _length (- _length 256)))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass lightrover_ros::Wrc201MsgResponse
  :super ros::object
  :slots (_readData ))

(defmethod lightrover_ros::Wrc201MsgResponse
  (:init
   (&key
    ((:readData __readData) 0)
    )
   (send-super :init)
   (setq _readData (round __readData))
   self)
  (:readData
   (&optional __readData)
   (if __readData (setq _readData __readData)) _readData)
  (:serialization-length
   ()
   (+
    ;; int64 _readData
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _readData
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _readData (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _readData) (= (length (_readData . bv)) 2)) ;; bignum
              (write-long (ash (elt (_readData . bv) 0) 0) s)
              (write-long (ash (elt (_readData . bv) 1) -1) s))
             ((and (class _readData) (= (length (_readData . bv)) 1)) ;; big1
              (write-long (elt (_readData . bv) 0) s)
              (write-long (if (>= _readData 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _readData s)(write-long (if (>= _readData 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _readData
#+(or :alpha :irix6 :x86_64)
      (setf _readData (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _readData (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass lightrover_ros::Wrc201Msg
  :super ros::object
  :slots ())

(setf (get lightrover_ros::Wrc201Msg :md5sum-) "43791c45179089218511643bd58fdb58")
(setf (get lightrover_ros::Wrc201Msg :datatype-) "lightrover_ros/Wrc201Msg")
(setf (get lightrover_ros::Wrc201Msg :request) lightrover_ros::Wrc201MsgRequest)
(setf (get lightrover_ros::Wrc201Msg :response) lightrover_ros::Wrc201MsgResponse)

(defmethod lightrover_ros::Wrc201MsgRequest
  (:response () (instance lightrover_ros::Wrc201MsgResponse :init)))

(setf (get lightrover_ros::Wrc201MsgRequest :md5sum-) "43791c45179089218511643bd58fdb58")
(setf (get lightrover_ros::Wrc201MsgRequest :datatype-) "lightrover_ros/Wrc201MsgRequest")
(setf (get lightrover_ros::Wrc201MsgRequest :definition-)
      "int8 addr
int64 data
int8 length
string cmd
---
int64 readData

")

(setf (get lightrover_ros::Wrc201MsgResponse :md5sum-) "43791c45179089218511643bd58fdb58")
(setf (get lightrover_ros::Wrc201MsgResponse :datatype-) "lightrover_ros/Wrc201MsgResponse")
(setf (get lightrover_ros::Wrc201MsgResponse :definition-)
      "int8 addr
int64 data
int8 length
string cmd
---
int64 readData

")



(provide :lightrover_ros/Wrc201Msg "43791c45179089218511643bd58fdb58")


