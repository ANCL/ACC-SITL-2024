; Auto-generated. Do not edit!


(cl:in-package offboardholy-msg)


;//! \htmlinclude PTStates.msg.html

(cl:defclass <PTStates> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (PT_states
    :reader PT_states
    :initarg :PT_states
    :type (cl:vector cl:float)
   :initform (cl:make-array 10 :element-type 'cl:float :initial-element 0.0))
   (ref_states
    :reader ref_states
    :initarg :ref_states
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PTStates (<PTStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PTStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PTStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name offboardholy-msg:<PTStates> is deprecated: use offboardholy-msg:PTStates instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PTStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offboardholy-msg:header-val is deprecated.  Use offboardholy-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'PT_states-val :lambda-list '(m))
(cl:defmethod PT_states-val ((m <PTStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offboardholy-msg:PT_states-val is deprecated.  Use offboardholy-msg:PT_states instead.")
  (PT_states m))

(cl:ensure-generic-function 'ref_states-val :lambda-list '(m))
(cl:defmethod ref_states-val ((m <PTStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader offboardholy-msg:ref_states-val is deprecated.  Use offboardholy-msg:ref_states instead.")
  (ref_states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PTStates>) ostream)
  "Serializes a message object of type '<PTStates>"
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
   (cl:slot-value msg 'PT_states))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'ref_states))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PTStates>) istream)
  "Deserializes a message object of type '<PTStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'PT_states) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'PT_states)))
    (cl:dotimes (i 10)
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
  (cl:setf (cl:slot-value msg 'ref_states) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'ref_states)))
    (cl:dotimes (i 4)
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PTStates>)))
  "Returns string type for a message object of type '<PTStates>"
  "offboardholy/PTStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PTStates)))
  "Returns string type for a message object of type 'PTStates"
  "offboardholy/PTStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PTStates>)))
  "Returns md5sum for a message object of type '<PTStates>"
  "e59142698ebb422e69c6931ba7839194")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PTStates)))
  "Returns md5sum for a message object of type 'PTStates"
  "e59142698ebb422e69c6931ba7839194")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PTStates>)))
  "Returns full string definition for message of type '<PTStates>"
  (cl:format cl:nil "#sls_state~%std_msgs/Header header~%float64[10] PT_states~%float64[4] ref_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PTStates)))
  "Returns full string definition for message of type 'PTStates"
  (cl:format cl:nil "#sls_state~%std_msgs/Header header~%float64[10] PT_states~%float64[4] ref_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PTStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'PT_states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ref_states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PTStates>))
  "Converts a ROS message object to a list"
  (cl:list 'PTStates
    (cl:cons ':header (header msg))
    (cl:cons ':PT_states (PT_states msg))
    (cl:cons ':ref_states (ref_states msg))
))
