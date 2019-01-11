
(cl:in-package :asdf)

(defsystem "light_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LightSrv" :depends-on ("_package_LightSrv"))
    (:file "_package_LightSrv" :depends-on ("_package"))
  ))