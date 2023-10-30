
(cl:in-package :asdf)

(defsystem "lightrover_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "Wrc201Msg" :depends-on ("_package_Wrc201Msg"))
    (:file "_package_Wrc201Msg" :depends-on ("_package"))
  ))