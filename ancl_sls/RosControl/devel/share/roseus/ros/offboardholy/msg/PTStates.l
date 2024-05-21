;; Auto-generated. Do not edit!


(when (boundp 'offboardholy::PTStates)
  (if (not (find-package "OFFBOARDHOLY"))
    (make-package "OFFBOARDHOLY"))
  (shadow 'PTStates (find-package "OFFBOARDHOLY")))
(unless (find-package "OFFBOARDHOLY::PTSTATES")
  (make-package "OFFBOARDHOLY::PTSTATES"))

(in-package "ROS")
;;//! \htmlinclude PTStates.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass offboardholy::PTStates
  :super ros::object
  :slots (_header _PT_states _ref_states ))

(defmethod offboardholy::PTStates
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:PT_states __PT_states) (make-array 10 :initial-element 0.0 :element-type :float))
    ((:ref_states __ref_states) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _PT_states __PT_states)
   (setq _ref_states __ref_states)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:PT_states
   (&optional __PT_states)
   (if __PT_states (setq _PT_states __PT_states)) _PT_states)
  (:ref_states
   (&optional __ref_states)
   (if __ref_states (setq _ref_states __ref_states)) _ref_states)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[10] _PT_states
    (* 8    10)
    ;; float64[4] _ref_states
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[10] _PT_states
     (dotimes (i 10)
       (sys::poke (elt _PT_states i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _ref_states
     (dotimes (i 4)
       (sys::poke (elt _ref_states i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[10] _PT_states
   (dotimes (i (length _PT_states))
     (setf (elt _PT_states i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _ref_states
   (dotimes (i (length _ref_states))
     (setf (elt _ref_states i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get offboardholy::PTStates :md5sum-) "e59142698ebb422e69c6931ba7839194")
(setf (get offboardholy::PTStates :datatype-) "offboardholy/PTStates")
(setf (get offboardholy::PTStates :definition-)
      "#sls_state
std_msgs/Header header
float64[10] PT_states
float64[4] ref_states

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :offboardholy/PTStates "e59142698ebb422e69c6931ba7839194")


