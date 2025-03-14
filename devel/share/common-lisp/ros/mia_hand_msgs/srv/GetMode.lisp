; Auto-generated. Do not edit!


(cl:in-package mia_hand_msgs-srv)


;//! \htmlinclude GetMode-request.msg.html

(cl:defclass <GetMode-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetMode-request (<GetMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-srv:<GetMode-request> is deprecated: use mia_hand_msgs-srv:GetMode-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMode-request>) ostream)
  "Serializes a message object of type '<GetMode-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMode-request>) istream)
  "Deserializes a message object of type '<GetMode-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMode-request>)))
  "Returns string type for a service object of type '<GetMode-request>"
  "mia_hand_msgs/GetModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMode-request)))
  "Returns string type for a service object of type 'GetMode-request"
  "mia_hand_msgs/GetModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMode-request>)))
  "Returns md5sum for a message object of type '<GetMode-request>"
  "bfda0693d1611db496d948702503f0f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMode-request)))
  "Returns md5sum for a message object of type 'GetMode-request"
  "bfda0693d1611db496d948702503f0f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMode-request>)))
  "Returns full string definition for message of type '<GetMode-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMode-request)))
  "Returns full string definition for message of type 'GetMode-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMode-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMode-request
))
;//! \htmlinclude GetMode-response.msg.html

(cl:defclass <GetMode-response> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetMode-response (<GetMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-srv:<GetMode-response> is deprecated: use mia_hand_msgs-srv:GetMode-response instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <GetMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-srv:mode-val is deprecated.  Use mia_hand_msgs-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GetMode-response>)))
    "Constants for message type '<GetMode-response>"
  '((:NORMAL_OPERATION . 0)
    (:PAUSED . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GetMode-response)))
    "Constants for message type 'GetMode-response"
  '((:NORMAL_OPERATION . 0)
    (:PAUSED . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMode-response>) ostream)
  "Serializes a message object of type '<GetMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMode-response>) istream)
  "Deserializes a message object of type '<GetMode-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMode-response>)))
  "Returns string type for a service object of type '<GetMode-response>"
  "mia_hand_msgs/GetModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMode-response)))
  "Returns string type for a service object of type 'GetMode-response"
  "mia_hand_msgs/GetModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMode-response>)))
  "Returns md5sum for a message object of type '<GetMode-response>"
  "bfda0693d1611db496d948702503f0f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMode-response)))
  "Returns md5sum for a message object of type 'GetMode-response"
  "bfda0693d1611db496d948702503f0f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMode-response>)))
  "Returns full string definition for message of type '<GetMode-response>"
  (cl:format cl:nil "uint8 NORMAL_OPERATION = 0~%uint8 PAUSED = 1~%~%uint8 mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMode-response)))
  "Returns full string definition for message of type 'GetMode-response"
  (cl:format cl:nil "uint8 NORMAL_OPERATION = 0~%uint8 PAUSED = 1~%~%uint8 mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMode-response
    (cl:cons ':mode (mode msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMode)))
  'GetMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMode)))
  'GetMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMode)))
  "Returns string type for a service object of type '<GetMode>"
  "mia_hand_msgs/GetMode")