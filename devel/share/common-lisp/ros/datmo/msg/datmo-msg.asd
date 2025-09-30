
(cl:in-package :asdf)

(defsystem "datmo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "ThetaPost" :depends-on ("_package_ThetaPost"))
    (:file "_package_ThetaPost" :depends-on ("_package"))
    (:file "Track" :depends-on ("_package_Track"))
    (:file "_package_Track" :depends-on ("_package"))
    (:file "TrackArray" :depends-on ("_package_TrackArray"))
    (:file "_package_TrackArray" :depends-on ("_package"))
  ))