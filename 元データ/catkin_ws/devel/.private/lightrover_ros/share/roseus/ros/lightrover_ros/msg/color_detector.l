;; Auto-generated. Do not edit!


(when (boundp 'lightrover_ros::color_detector)
  (if (not (find-package "LIGHTROVER_ROS"))
    (make-package "LIGHTROVER_ROS"))
  (shadow 'color_detector (find-package "LIGHTROVER_ROS")))
(unless (find-package "LIGHTROVER_ROS::COLOR_DETECTOR")
  (make-package "LIGHTROVER_ROS::COLOR_DETECTOR"))

(in-package "ROS")
;;//! \htmlinclude color_detector.msg.html


(defclass lightrover_ros::color_detector
  :super ros::object
  :slots (_color_type ))

(defmethod lightrover_ros::color_detector
  (:init
   (&key
    ((:color_type __color_type) 0)
    )
   (send-super :init)
   (setq _color_type (round __color_type))
   self)
  (:color_type
   (&optional __color_type)
   (if __color_type (setq _color_type __color_type)) _color_type)
  (:serialization-length
   ()
   (+
    ;; int8 _color_type
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _color_type
       (write-byte _color_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _color_type
     (setq _color_type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _color_type 127) (setq _color_type (- _color_type 256)))
   ;;
   self)
  )

(setf (get lightrover_ros::color_detector :md5sum-) "28ff8e773f1e211eaff66787abb1695e")
(setf (get lightrover_ros::color_detector :datatype-) "lightrover_ros/color_detector")
(setf (get lightrover_ros::color_detector :definition-)
      "int8 color_type

")



(provide :lightrover_ros/color_detector "28ff8e773f1e211eaff66787abb1695e")


