;; Auto-generated. Do not edit!


(when (boundp 'datmo::ThetaPost)
  (if (not (find-package "DATMO"))
    (make-package "DATMO"))
  (shadow 'ThetaPost (find-package "DATMO")))
(unless (find-package "DATMO::THETAPOST")
  (make-package "DATMO::THETAPOST"))

(in-package "ROS")
;;//! \htmlinclude ThetaPost.msg.html


(defclass datmo::ThetaPost
  :super ros::object
  :slots (_theta _distance ))

(defmethod datmo::ThetaPost
  (:init
   (&key
    ((:theta __theta) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:distance __distance) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _theta __theta)
   (setq _distance __distance)
   self)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float32[] _theta
    (* 4    (length _theta)) 4
    ;; float32[] _distance
    (* 4    (length _distance)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _theta
     (write-long (length _theta) s)
     (dotimes (i (length _theta))
       (sys::poke (elt _theta i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _distance
     (write-long (length _distance) s)
     (dotimes (i (length _distance))
       (sys::poke (elt _distance i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _theta
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _theta (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _theta i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _distance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _distance i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get datmo::ThetaPost :md5sum-) "4c4ad3bc73961ea47fc04a581b96a79c")
(setf (get datmo::ThetaPost :datatype-) "datmo/ThetaPost")
(setf (get datmo::ThetaPost :definition-)
      "float32[] theta
float32[] distance
")



(provide :datmo/ThetaPost "4c4ad3bc73961ea47fc04a581b96a79c")


