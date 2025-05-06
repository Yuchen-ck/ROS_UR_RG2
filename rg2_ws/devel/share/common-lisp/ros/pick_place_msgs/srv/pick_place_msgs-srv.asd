
(cl:in-package :asdf)

(defsystem "pick_place_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PickStep" :depends-on ("_package_PickStep"))
    (:file "_package_PickStep" :depends-on ("_package"))
  ))