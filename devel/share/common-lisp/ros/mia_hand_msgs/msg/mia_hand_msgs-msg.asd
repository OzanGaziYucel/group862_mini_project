
(cl:in-package :asdf)

(defsystem "mia_hand_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ComponentStatus" :depends-on ("_package_ComponentStatus"))
    (:file "_package_ComponentStatus" :depends-on ("_package"))
    (:file "FingersData" :depends-on ("_package_FingersData"))
    (:file "_package_FingersData" :depends-on ("_package"))
    (:file "FingersStrainGauges" :depends-on ("_package_FingersStrainGauges"))
    (:file "_package_FingersStrainGauges" :depends-on ("_package"))
    (:file "GraspRef" :depends-on ("_package_GraspRef"))
    (:file "_package_GraspRef" :depends-on ("_package"))
  ))