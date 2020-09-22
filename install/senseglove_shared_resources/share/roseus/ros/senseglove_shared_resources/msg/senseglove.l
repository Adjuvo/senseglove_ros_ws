;; Auto-generated. Do not edit!


(when (boundp 'senseglove_shared_resources::senseglove)
  (if (not (find-package "SENSEGLOVE_SHARED_RESOURCES"))
    (make-package "SENSEGLOVE_SHARED_RESOURCES"))
  (shadow 'senseglove (find-package "SENSEGLOVE_SHARED_RESOURCES")))
(unless (find-package "SENSEGLOVE_SHARED_RESOURCES::SENSEGLOVE")
  (make-package "SENSEGLOVE_SHARED_RESOURCES::SENSEGLOVE"))

(in-package "ROS")
;;//! \htmlinclude senseglove.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass senseglove_shared_resources::senseglove
  :super ros::object
  :slots (_header _data1 ))

(defmethod senseglove_shared_resources::senseglove
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data1 __data1) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _data1 (float __data1))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data1
   (&optional __data1)
   (if __data1 (setq _data1 __data1)) _data1)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _data1
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _data1
       (sys::poke _data1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _data1
     (setq _data1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get senseglove_shared_resources::senseglove :md5sum-) "2a7bc7e8311daf6b11da471ec94dadf1")
(setf (get senseglove_shared_resources::senseglove :datatype-) "senseglove_shared_resources/senseglove")
(setf (get senseglove_shared_resources::senseglove :definition-)
      "# A message for senseglove data
Header header

float64 data1
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



(provide :senseglove_shared_resources/senseglove "2a7bc7e8311daf6b11da471ec94dadf1")


