; Auto-generated. Do not edit!


(cl:in-package senseglove_shared_resources-msg)


;//! \htmlinclude senseglove.msg.html

(cl:defclass <senseglove> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data1
    :reader data1
    :initarg :data1
    :type cl:float
    :initform 0.0))
)

(cl:defclass senseglove (<senseglove>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <senseglove>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'senseglove)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name senseglove_shared_resources-msg:<senseglove> is deprecated: use senseglove_shared_resources-msg:senseglove instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <senseglove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader senseglove_shared_resources-msg:header-val is deprecated.  Use senseglove_shared_resources-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data1-val :lambda-list '(m))
(cl:defmethod data1-val ((m <senseglove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader senseglove_shared_resources-msg:data1-val is deprecated.  Use senseglove_shared_resources-msg:data1 instead.")
  (data1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <senseglove>) ostream)
  "Serializes a message object of type '<senseglove>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <senseglove>) istream)
  "Deserializes a message object of type '<senseglove>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data1) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<senseglove>)))
  "Returns string type for a message object of type '<senseglove>"
  "senseglove_shared_resources/senseglove")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'senseglove)))
  "Returns string type for a message object of type 'senseglove"
  "senseglove_shared_resources/senseglove")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<senseglove>)))
  "Returns md5sum for a message object of type '<senseglove>"
  "2a7bc7e8311daf6b11da471ec94dadf1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'senseglove)))
  "Returns md5sum for a message object of type 'senseglove"
  "2a7bc7e8311daf6b11da471ec94dadf1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<senseglove>)))
  "Returns full string definition for message of type '<senseglove>"
  (cl:format cl:nil "# A message for senseglove data~%Header header~%~%float64 data1~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'senseglove)))
  "Returns full string definition for message of type 'senseglove"
  (cl:format cl:nil "# A message for senseglove data~%Header header~%~%float64 data1~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <senseglove>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <senseglove>))
  "Converts a ROS message object to a list"
  (cl:list 'senseglove
    (cl:cons ':header (header msg))
    (cl:cons ':data1 (data1 msg))
))
