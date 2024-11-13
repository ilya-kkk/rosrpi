
(cl:in-package :asdf)

(defsystem "my_super_robot_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "my_Num" :depends-on ("_package_my_Num"))
    (:file "_package_my_Num" :depends-on ("_package"))
  ))