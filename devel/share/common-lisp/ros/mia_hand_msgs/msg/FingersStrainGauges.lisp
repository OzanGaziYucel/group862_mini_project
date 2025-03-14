; Auto-generated. Do not edit!


(cl:in-package mia_hand_msgs-msg)


;//! \htmlinclude FingersStrainGauges.msg.html

(cl:defclass <FingersStrainGauges> (roslisp-msg-protocol:ros-message)
  ((thu
    :reader thu
    :initarg :thu
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 2 :element-type 'cl:fixnum :initial-element 0))
   (ind
    :reader ind
    :initarg :ind
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 2 :element-type 'cl:fixnum :initial-element 0))
   (mrl
    :reader mrl
    :initarg :mrl
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 2 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass FingersStrainGauges (<FingersStrainGauges>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FingersStrainGauges>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FingersStrainGauges)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mia_hand_msgs-msg:<FingersStrainGauges> is deprecated: use mia_hand_msgs-msg:FingersStrainGauges instead.")))

(cl:ensure-generic-function 'thu-val :lambda-list '(m))
(cl:defmethod thu-val ((m <FingersStrainGauges>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:thu-val is deprecated.  Use mia_hand_msgs-msg:thu instead.")
  (thu m))

(cl:ensure-generic-function 'ind-val :lambda-list '(m))
(cl:defmethod ind-val ((m <FingersStrainGauges>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:ind-val is deprecated.  Use mia_hand_msgs-msg:ind instead.")
  (ind m))

(cl:ensure-generic-function 'mrl-val :lambda-list '(m))
(cl:defmethod mrl-val ((m <FingersStrainGauges>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mia_hand_msgs-msg:mrl-val is deprecated.  Use mia_hand_msgs-msg:mrl instead.")
  (mrl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FingersStrainGauges>) ostream)
  "Serializes a message object of type '<FingersStrainGauges>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'thu))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'ind))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'mrl))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FingersStrainGauges>) istream)
  "Deserializes a message object of type '<FingersStrainGauges>"
  (cl:setf (cl:slot-value msg 'thu) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'thu)))
    (cl:dotimes (i 2)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'ind) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'ind)))
    (cl:dotimes (i 2)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'mrl) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'mrl)))
    (cl:dotimes (i 2)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FingersStrainGauges>)))
  "Returns string type for a message object of type '<FingersStrainGauges>"
  "mia_hand_msgs/FingersStrainGauges")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FingersStrainGauges)))
  "Returns string type for a message object of type 'FingersStrainGauges"
  "mia_hand_msgs/FingersStrainGauges")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FingersStrainGauges>)))
  "Returns md5sum for a message object of type '<FingersStrainGauges>"
  "8e0dffdebc67e94f504a570700ccd19d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FingersStrainGauges)))
  "Returns md5sum for a message object of type 'FingersStrainGauges"
  "8e0dffdebc67e94f504a570700ccd19d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FingersStrainGauges>)))
  "Returns full string definition for message of type '<FingersStrainGauges>"
  (cl:format cl:nil "int16[2] thu~%int16[2] ind~%int16[2] mrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FingersStrainGauges)))
  "Returns full string definition for message of type 'FingersStrainGauges"
  (cl:format cl:nil "int16[2] thu~%int16[2] ind~%int16[2] mrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FingersStrainGauges>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'thu) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ind) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mrl) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FingersStrainGauges>))
  "Converts a ROS message object to a list"
  (cl:list 'FingersStrainGauges
    (cl:cons ':thu (thu msg))
    (cl:cons ':ind (ind msg))
    (cl:cons ':mrl (mrl msg))
))
