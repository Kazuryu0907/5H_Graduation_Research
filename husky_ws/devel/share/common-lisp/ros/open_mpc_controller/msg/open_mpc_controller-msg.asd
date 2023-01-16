
(cl:in-package :asdf)

(defsystem "open_mpc_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OptimizationParameters" :depends-on ("_package_OptimizationParameters"))
    (:file "_package_OptimizationParameters" :depends-on ("_package"))
    (:file "OptimizationResult" :depends-on ("_package_OptimizationResult"))
    (:file "_package_OptimizationResult" :depends-on ("_package"))
  ))