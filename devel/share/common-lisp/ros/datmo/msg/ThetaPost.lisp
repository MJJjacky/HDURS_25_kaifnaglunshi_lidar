; Auto-generated. Do not edit!


(cl:in-package datmo-msg)


;//! \htmlinclude ThetaPost.msg.html

(cl:defclass <ThetaPost> (roslisp-msg-protocol:ros-message)
  ((theta
    :reader theta
    :initarg :theta
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (distance
    :reader distance
    :initarg :distance
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ThetaPost (<ThetaPost>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThetaPost>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThetaPost)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name datmo-msg:<ThetaPost> is deprecated: use datmo-msg:ThetaPost instead.")))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <ThetaPost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader datmo-msg:theta-val is deprecated.  Use datmo-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <ThetaPost>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader datmo-msg:distance-val is deprecated.  Use datmo-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThetaPost>) ostream)
  "Serializes a message object of type '<ThetaPost>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'theta))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'distance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThetaPost>) istream)
  "Deserializes a message object of type '<ThetaPost>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'theta) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'theta)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'distance) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distance)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThetaPost>)))
  "Returns string type for a message object of type '<ThetaPost>"
  "datmo/ThetaPost")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThetaPost)))
  "Returns string type for a message object of type 'ThetaPost"
  "datmo/ThetaPost")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThetaPost>)))
  "Returns md5sum for a message object of type '<ThetaPost>"
  "4c4ad3bc73961ea47fc04a581b96a79c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThetaPost)))
  "Returns md5sum for a message object of type 'ThetaPost"
  "4c4ad3bc73961ea47fc04a581b96a79c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThetaPost>)))
  "Returns full string definition for message of type '<ThetaPost>"
  (cl:format cl:nil "float32[] theta~%float32[] distance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThetaPost)))
  "Returns full string definition for message of type 'ThetaPost"
  (cl:format cl:nil "float32[] theta~%float32[] distance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThetaPost>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'theta) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThetaPost>))
  "Converts a ROS message object to a list"
  (cl:list 'ThetaPost
    (cl:cons ':theta (theta msg))
    (cl:cons ':distance (distance msg))
))
