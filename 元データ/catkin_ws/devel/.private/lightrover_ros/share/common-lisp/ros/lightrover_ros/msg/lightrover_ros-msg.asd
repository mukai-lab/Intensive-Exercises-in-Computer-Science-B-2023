
(cl:in-package :asdf)

(defsystem "lightrover_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "color_detector" :depends-on ("_package_color_detector"))
    (:file "_package_color_detector" :depends-on ("_package"))
  ))