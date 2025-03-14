; Auto-generated. Do not edit!


(cl:in-package mia_hand_msgs-msg)


;//! \htmlinclude FingersData.msg.html

(cl:defclass <FingersData> (roslisp-msg-protocol:ros-message)
  ((thu
    :reader thu
    :initarg :thu
    :type cl:fixnum
    :initform 0)
   (ind
    :reader ind
    :initarg :ind
    :type cl:fixnum
    :initform 0)
   (mrl
    :reader mrl
    :initarg :mrl
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FingersData (<FingersData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FingersData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FingersData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-msg:<FingersData> is deprecated: use mia_hand_msgs-msg:FingersData instead.")))

(cl:ensure-generic-function 'thu-val :lambda-list '(m))
(cl:defmethod thu-val ((m <FingersData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:thu-val is deprecated.  Use mia_hand_msgs-msg:thu instead.")
  (thu m))

(cl:ensure-generic-function 'ind-val :lambda-list '(m))
(cl:defmethod ind-val ((m <FingersData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:ind-val is deprecated.  Use mia_hand_msgs-msg:ind instead.")
  (ind m))

(cl:ensure-generic-function 'mrl-val :lambda-list '(m))
(cl:defmethod mrl-val ((m <FingersData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:mrl-val is deprecated.  Use mia_hand_msgs-msg:mrl instead.")
  (mrl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FingersData>) ostream)
  "Serializes a message object of type '<FingersData>"
  (cl:let* ((signed (cl:slot-value msg 'thu)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ind)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mrl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FingersData>) istream)
  "Deserializes a message object of type '<FingersData>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'thu) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ind) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mrl) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FingersData>)))
  "Returns string type for a message object of type '<FingersData>"
  "mia_hand_msgs/FingersData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingersData)))
  "Returns string type for a message object of type 'FingersData"
  "mia_hand_msgs/FingersData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FingersData>)))
  "Returns md5sum for a message object of type '<FingersData>"
  "67f7588122a2649383b5219661ab68d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FingersData)))
  "Returns md5sum for a message object of type 'FingersData"
  "67f7588122a2649383b5219661ab68d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FingersData>)))
  "Returns full string definition for message of type '<FingersData>"
  (cl:format cl:nil "int16 thu~%int16 ind~%int16 mrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FingersData)))
  "Returns full string definition for message of type 'FingersData"
  (cl:format cl:nil "int16 thu~%int16 ind~%int16 mrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FingersData>))
  (cl:+ 0
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FingersData>))
  "Converts a ROS message object to a list"
  (cl:list 'FingersData
    (cl:cons ':thu (thu msg))
    (cl:cons ':ind (ind msg))
    (cl:cons ':mrl (mrl msg))
))
