
(cl:in-package :asdf)

(defsystem "package1234-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "dest" :depends-on ("_package_dest"))
    (:file "_package_dest" :depends-on ("_package"))
  ))