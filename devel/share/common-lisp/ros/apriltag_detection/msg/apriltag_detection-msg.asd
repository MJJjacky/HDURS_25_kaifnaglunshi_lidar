
(cl:in-package :asdf)

(defsystem "apriltag_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AprilTagDistance" :depends-on ("_package_AprilTagDistance"))
    (:file "_package_AprilTagDistance" :depends-on ("_package"))
  ))