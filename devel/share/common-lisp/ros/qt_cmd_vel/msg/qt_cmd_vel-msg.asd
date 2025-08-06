
(cl:in-package :asdf)

(defsystem "qt_cmd_vel-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "personData" :depends-on ("_package_personData"))
    (:file "_package_personData" :depends-on ("_package"))
  ))