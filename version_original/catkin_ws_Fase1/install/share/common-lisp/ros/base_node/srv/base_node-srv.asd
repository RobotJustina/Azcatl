
(cl:in-package :asdf)

(defsystem "base_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OverSrv" :depends-on ("_package_OverSrv"))
    (:file "_package_OverSrv" :depends-on ("_package"))
    (:file "MVServ" :depends-on ("_package_MVServ"))
    (:file "_package_MVServ" :depends-on ("_package"))
  ))