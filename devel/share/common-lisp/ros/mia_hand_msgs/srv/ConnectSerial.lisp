; Auto-generated. Do not edit!


(cl:in-package mia_hand_msgs-srv)


;//! \htmlinclude ConnectSerial-request.msg.html

(cl:defclass <ConnectSerial-request> (roslisp-msg-protocol:ros-message)
  ((port
    :reader port
    :initarg :port
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ConnectSerial-request (<ConnectSerial-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConnectSerial-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConnectSerial-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-srv:<ConnectSerial-request> is deprecated: use mia_hand_msgs-srv:ConnectSerial-request instead.")))

(cl:ensure-generic-function 'port-val :lambda-list '(m))
(cl:defmethod port-val ((m <ConnectSerial-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-srv:port-val is deprecated.  Use mia_hand_msgs-srv:port instead.")
  (port m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConnectSerial-request>) ostream)
  "Serializes a message object of type '<ConnectSerial-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConnectSerial-request>) istream)
  "Deserializes a message object of type '<ConnectSerial-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'port)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'port)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConnectSerial-request>)))
  "Returns string type for a service object of type '<ConnectSerial-request>"
  "mia_hand_msgs/ConnectSerialRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectSerial-request)))
  "Returns string type for a service object of type 'ConnectSerial-request"
  "mia_hand_msgs/ConnectSerialRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConnectSerial-request>)))
  "Returns md5sum for a message object of type '<ConnectSerial-request>"
  "86e89cea09faaef62a0ac1903573e480")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConnectSerial-request)))
  "Returns md5sum for a message object of type 'ConnectSerial-request"
  "86e89cea09faaef62a0ac1903573e480")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConnectSerial-request>)))
  "Returns full string definition for message of type '<ConnectSerial-request>"
  (cl:format cl:nil "uint16 port~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConnectSerial-request)))
  "Returns full string definition for message of type 'ConnectSerial-request"
  (cl:format cl:nil "uint16 port~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConnectSerial-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConnectSerial-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConnectSerial-request
    (cl:cons ':port (port msg))
))
;//! \htmlinclude ConnectSerial-response.msg.html

(cl:defclass <ConnectSerial-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ConnectSerial-response (<ConnectSerial-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConnectSerial-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConnectSerial-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-srv:<ConnectSerial-response> is deprecated: use mia_hand_msgs-srv:ConnectSerial-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ConnectSerial-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-srv:success-val is deprecated.  Use mia_hand_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ConnectSerial-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-srv:message-val is deprecated.  Use mia_hand_msgs-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConnectSerial-response>) ostream)
  "Serializes a message object of type '<ConnectSerial-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConnectSerial-response>) istream)
  "Deserializes a message object of type '<ConnectSerial-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConnectSerial-response>)))
  "Returns string type for a service object of type '<ConnectSerial-response>"
  "mia_hand_msgs/ConnectSerialResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectSerial-response)))
  "Returns string type for a service object of type 'ConnectSerial-response"
  "mia_hand_msgs/ConnectSerialResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConnectSerial-response>)))
  "Returns md5sum for a message object of type '<ConnectSerial-response>"
  "86e89cea09faaef62a0ac1903573e480")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConnectSerial-response)))
  "Returns md5sum for a message object of type 'ConnectSerial-response"
  "86e89cea09faaef62a0ac1903573e480")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConnectSerial-response>)))
  "Returns full string definition for message of type '<ConnectSerial-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConnectSerial-response)))
  "Returns full string definition for message of type 'ConnectSerial-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConnectSerial-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConnectSerial-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConnectSerial-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConnectSerial)))
  'ConnectSerial-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConnectSerial)))
  'ConnectSerial-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConnectSerial)))
  "Returns string type for a service object of type '<ConnectSerial>"
  "mia_hand_msgs/ConnectSerial")