; Auto-generated. Do not edit!


(cl:in-package mia_hand_msgs-msg)


;//! \htmlinclude ComponentStatus.msg.html

(cl:defclass <ComponentStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil)
   (msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass ComponentStatus (<ComponentStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComponentStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComponentStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-msg:<ComponentStatus> is deprecated: use mia_hand_msgs-msg:ComponentStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ComponentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:header-val is deprecated.  Use mia_hand_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ComponentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:status-val is deprecated.  Use mia_hand_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <ComponentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:msg-val is deprecated.  Use mia_hand_msgs-msg:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComponentStatus>) ostream)
  "Serializes a message object of type '<ComponentStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComponentStatus>) istream)
  "Deserializes a message object of type '<ComponentStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComponentStatus>)))
  "Returns string type for a message object of type '<ComponentStatus>"
  "mia_hand_msgs/ComponentStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComponentStatus)))
  "Returns string type for a message object of type 'ComponentStatus"
  "mia_hand_msgs/ComponentStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComponentStatus>)))
  "Returns md5sum for a message object of type '<ComponentStatus>"
  "365c676b4aefb93cfc7b974237335cea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComponentStatus)))
  "Returns md5sum for a message object of type 'ComponentStatus"
  "365c676b4aefb93cfc7b974237335cea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComponentStatus>)))
  "Returns full string definition for message of type '<ComponentStatus>"
  (cl:format cl:nil "std_msgs/Header header~%bool status             # True means component is ok~%string msg              # Error message (when state is set to False)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComponentStatus)))
  "Returns full string definition for message of type 'ComponentStatus"
  (cl:format cl:nil "std_msgs/Header header~%bool status             # True means component is ok~%string msg              # Error message (when state is set to False)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComponentStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComponentStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ComponentStatus
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
    (cl:cons ':msg (msg msg))
))
