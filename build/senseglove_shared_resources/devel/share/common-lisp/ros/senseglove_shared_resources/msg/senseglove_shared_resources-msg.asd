
(cl:in-package :asdf)

(defsystem "senseglove_shared_resources-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "senseglove" :depends-on ("_package_senseglove"))
    (:file "_package_senseglove" :depends-on ("_package"))
  ))