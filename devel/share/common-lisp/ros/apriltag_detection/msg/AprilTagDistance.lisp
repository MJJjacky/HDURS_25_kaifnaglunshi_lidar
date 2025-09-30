; Auto-generated. Do not edit!


(cl:in-package apriltag_detection-msg)


;//! \htmlinclude AprilTagDistance.msg.html

(cl:defclass <AprilTagDistance> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass AprilTagDistance (<AprilTagDistance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AprilTagDistance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AprilTagDistance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltag_detection-msg:<AprilTagDistance> is deprecated: use apriltag_detection-msg:AprilTagDistance instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <AprilTagDistance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltag_detection-msg:distance-val is deprecated.  Use apriltag_detection-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AprilTagDistance>) ostream)
  "Serializes a message object of type '<AprilTagDistance>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AprilTagDistance>) istream)
  "Deserializes a message object of type '<AprilTagDistance>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AprilTagDistance>)))
  "Returns string type for a message object of type '<AprilTagDistance>"
  "apriltag_detection/AprilTagDistance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AprilTagDistance)))
  "Returns string type for a message object of type 'AprilTagDistance"
  "apriltag_detection/AprilTagDistance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AprilTagDistance>)))
  "Returns md5sum for a message object of type '<AprilTagDistance>"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AprilTagDistance)))
  "Returns md5sum for a message object of type 'AprilTagDistance"
  "6e77fb10f0c8b4833ec273aa9ac74459")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AprilTagDistance>)))
  "Returns full string definition for message of type '<AprilTagDistance>"
  (cl:format cl:nil "float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AprilTagDistance)))
  "Returns full string definition for message of type 'AprilTagDistance"
  (cl:format cl:nil "float32 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AprilTagDistance>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AprilTagDistance>))
  "Converts a ROS message object to a list"
  (cl:list 'AprilTagDistance
    (cl:cons ':distance (distance msg))
))
