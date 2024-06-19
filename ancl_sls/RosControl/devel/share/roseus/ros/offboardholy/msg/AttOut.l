;; Auto-generated. Do not edit!


(when (boundp 'offboardholy::AttOut)
  (if (not (find-package "OFFBOARDHOLY"))
    (make-package "OFFBOARDHOLY"))
  (shadow 'AttOut (find-package "OFFBOARDHOLY")))
(unless (find-package "OFFBOARDHOLY::ATTOUT")
  (make-package "OFFBOARDHOLY::ATTOUT"))

(in-package "ROS")
;;//! \htmlinclude AttOut.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass offboardholy::AttOut
  :super ros::object
  :slots (_header _rpy _con_out _target_thrust ))

(defmethod offboardholy::AttOut
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rpy __rpy) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:con_out __con_out) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:target_thrust __target_thrust) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _rpy __rpy)
   (setq _con_out __con_out)
   (setq _target_thrust (float __target_thrust))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rpy
   (&optional __rpy)
   (if __rpy (setq _rpy __rpy)) _rpy)
  (:con_out
   (&optional __con_out)
   (if __con_out (setq _con_out __con_out)) _con_out)
  (:target_thrust
   (&optional __target_thrust)
   (if __target_thrust (setq _target_thrust __target_thrust)) _target_thrust)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[3] _rpy
    (* 8    3)
    ;; float64[3] _con_out
    (* 8    3)
    ;; float64 _target_thrust
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[3] _rpy
     (dotimes (i 3)
       (sys::poke (elt _rpy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _con_out
     (dotimes (i 3)
       (sys::poke (elt _con_out i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _target_thrust
       (sys::poke _target_thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[3] _rpy
   (dotimes (i (length _rpy))
     (setf (elt _rpy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _con_out
   (dotimes (i (length _con_out))
     (setf (elt _con_out i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _target_thrust
     (setq _target_thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get offboardholy::AttOut :md5sum-) "98ce28f293a4f5e66b12864b50c00965")
(setf (get offboardholy::AttOut :datatype-) "offboardholy/AttOut")
(setf (get offboardholy::AttOut :definition-)
      "#For attitude (roll, pitch, yaw) and controller outputs (3)
Header header
float64[3] rpy
float64[3] con_out
float64 target_thrust
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



(provide :offboardholy/AttOut "98ce28f293a4f5e66b12864b50c00965")


