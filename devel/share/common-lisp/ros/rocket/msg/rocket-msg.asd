
(cl:in-package :asdf)

(defsystem "rocket-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RosGPS" :depends-on ("_package_RosGPS"))
    (:file "_package_RosGPS" :depends-on ("_package"))
  ))