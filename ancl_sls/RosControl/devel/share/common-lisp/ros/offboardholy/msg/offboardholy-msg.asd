
(cl:in-package :asdf)

(defsystem "offboardholy-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ActuatorControl0" :depends-on ("_package_ActuatorControl0"))
    (:file "_package_ActuatorControl0" :depends-on ("_package"))
    (:file "PTStates" :depends-on ("_package_PTStates"))
    (:file "_package_PTStates" :depends-on ("_package"))
  ))