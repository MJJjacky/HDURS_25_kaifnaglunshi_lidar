;; Auto-generated. Do not edit!


(when (boundp 'apriltag_detection::AprilTagDistance)
  (if (not (find-package "APRILTAG_DETECTION"))
    (make-package "APRILTAG_DETECTION"))
  (shadow 'AprilTagDistance (find-package "APRILTAG_DETECTION")))
(unless (find-package "APRILTAG_DETECTION::APRILTAGDISTANCE")
  (make-package "APRILTAG_DETECTION::APRILTAGDISTANCE"))

(in-package "ROS")
;;//! \htmlinclude AprilTagDistance.msg.html


(defclass apriltag_detection::AprilTagDistance
  :super ros::object
  :slots (_distance ))

(defmethod apriltag_detection::AprilTagDistance
  (:init
   (&key
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get apriltag_detection::AprilTagDistance :md5sum-) "6e77fb10f0c8b4833ec273aa9ac74459")
(setf (get apriltag_detection::AprilTagDistance :datatype-) "apriltag_detection/AprilTagDistance")
(setf (get apriltag_detection::AprilTagDistance :definition-)
      "float32 distance

")



(provide :apriltag_detection/AprilTagDistance "6e77fb10f0c8b4833ec273aa9ac74459")


