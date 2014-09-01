
(cl:in-package :asdf)

(defsystem "rocket-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RosGPS" :depends-on ("_package_RosGPS"))
    (:file "_package_RosGPS" :depends-on ("_package"))
    (:file "Vector3Stamped" :depends-on ("_package_Vector3Stamped"))
    (:file "_package_Vector3Stamped" :depends-on ("_package"))
    (:file "Vector3" :depends-on ("_package_Vector3"))
    (:file "_package_Vector3" :depends-on ("_package"))
  ))