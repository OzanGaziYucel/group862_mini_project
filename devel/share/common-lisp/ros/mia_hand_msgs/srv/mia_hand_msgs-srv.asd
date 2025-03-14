
(cl:in-package :asdf)

(defsystem "mia_hand_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ConnectSerial" :depends-on ("_package_ConnectSerial"))
    (:file "_package_ConnectSerial" :depends-on ("_package"))
    (:file "GetMode" :depends-on ("_package_GetMode"))
    (:file "_package_GetMode" :depends-on ("_package"))
  ))