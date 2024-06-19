; Auto-generated. Do not edit!


(cl:in-package offboardholy-msg)


;//! \htmlinclude AttOut.msg.html

(cl:defclass <AttOut> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rpy
    :reader rpy
    :initarg :rpy
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (con_out
    :reader con_out
    :initarg :con_out
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (target_thrust
    :reader target_thrust
    :initarg :target_thrust
    :type cl:float
    :initform 0.0))
)

(cl:defclass AttOut (<AttOut>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttOut>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttOut)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name offboardholy-msg:<AttOut> is deprecated: use offboardholy-msg:AttOut instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AttOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offboardholy-msg:header-val is deprecated.  Use offboardholy-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rpy-val :lambda-list '(m))
(cl:defmethod rpy-val ((m <AttOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offboardholy-msg:rpy-val is deprecated.  Use offboardholy-msg:rpy instead.")
  (rpy m))

(cl:ensure-generic-function 'con_out-val :lambda-list '(m))
(cl:defmethod con_out-val ((m <AttOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offboardholy-msg:con_out-val is deprecated.  Use offboardholy-msg:con_out instead.")
  (con_out m))

(cl:ensure-generic-function 'target_thrust-val :lambda-list '(m))
(cl:defmethod target_thrust-val ((m <AttOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offboardholy-msg:target_thrust-val is deprecated.  Use offboardholy-msg:target_thrust instead.")
  (target_thrust m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttOut>) ostream)
  "Serializes a message object of type '<AttOut>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'rpy))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'con_out))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttOut>) istream)
  "Deserializes a message object of type '<AttOut>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'rpy) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'rpy)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'con_out) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'con_out)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_thrust) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttOut>)))
  "Returns string type for a message object of type '<AttOut>"
  "offboardholy/AttOut")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttOut)))
  "Returns string type for a message object of type 'AttOut"
  "offboardholy/AttOut")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttOut>)))
  "Returns md5sum for a message object of type '<AttOut>"
  "98ce28f293a4f5e66b12864b50c00965")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttOut)))
  "Returns md5sum for a message object of type 'AttOut"
  "98ce28f293a4f5e66b12864b50c00965")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttOut>)))
  "Returns full string definition for message of type '<AttOut>"
  (cl:format cl:nil "#For attitude (roll, pitch, yaw) and controller outputs (3)~%Header header~%float64[3] rpy~%float64[3] con_out~%float64 target_thrust~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttOut)))
  "Returns full string definition for message of type 'AttOut"
  (cl:format cl:nil "#For attitude (roll, pitch, yaw) and controller outputs (3)~%Header header~%float64[3] rpy~%float64[3] con_out~%float64 target_thrust~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttOut>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'rpy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'con_out) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttOut>))
  "Converts a ROS message object to a list"
  (cl:list 'AttOut
    (cl:cons ':header (header msg))
    (cl:cons ':rpy (rpy msg))
    (cl:cons ':con_out (con_out msg))
    (cl:cons ':target_thrust (target_thrust msg))
))
