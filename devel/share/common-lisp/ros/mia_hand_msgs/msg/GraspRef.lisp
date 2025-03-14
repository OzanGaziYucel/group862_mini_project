; Auto-generated. Do not edit!


(cl:in-package mia_hand_msgs-msg)


;//! \htmlinclude GraspRef.msg.html

(cl:defclass <GraspRef> (roslisp-msg-protocol:ros-message)
  ((rest
    :reader rest
    :initarg :rest
    :type cl:fixnum
    :initform 0)
   (pos
    :reader pos
    :initarg :pos
    :type cl:fixnum
    :initform 0)
   (delay
    :reader delay
    :initarg :delay
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GraspRef (<GraspRef>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspRef>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspRef)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-msg:<GraspRef> is deprecated: use mia_hand_msgs-msg:GraspRef instead.")))

(cl:ensure-generic-function 'rest-val :lambda-list '(m))
(cl:defmethod rest-val ((m <GraspRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:rest-val is deprecated.  Use mia_hand_msgs-msg:rest instead.")
  (rest m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <GraspRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:pos-val is deprecated.  Use mia_hand_msgs-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'delay-val :lambda-list '(m))
(cl:defmethod delay-val ((m <GraspRef>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:delay-val is deprecated.  Use mia_hand_msgs-msg:delay instead.")
  (delay m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspRef>) ostream)
  "Serializes a message object of type '<GraspRef>"
  (cl:let* ((signed (cl:slot-value msg 'rest)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'delay)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspRef>) istream)
  "Deserializes a message object of type '<GraspRef>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rest) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'delay) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspRef>)))
  "Returns string type for a message object of type '<GraspRef>"
  "mia_hand_msgs/GraspRef")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspRef)))
  "Returns string type for a message object of type 'GraspRef"
  "mia_hand_msgs/GraspRef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspRef>)))
  "Returns md5sum for a message object of type '<GraspRef>"
  "09f519c8adf07285290a71d280a83d8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspRef)))
  "Returns md5sum for a message object of type 'GraspRef"
  "09f519c8adf07285290a71d280a83d8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspRef>)))
  "Returns full string definition for message of type '<GraspRef>"
  (cl:format cl:nil "int16 rest~%int16 pos~%int16 delay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspRef)))
  "Returns full string definition for message of type 'GraspRef"
  (cl:format cl:nil "int16 rest~%int16 pos~%int16 delay~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspRef>))
  (cl:+ 0
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspRef>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspRef
    (cl:cons ':rest (rest msg))
    (cl:cons ':pos (pos msg))
    (cl:cons ':delay (delay msg))
))
