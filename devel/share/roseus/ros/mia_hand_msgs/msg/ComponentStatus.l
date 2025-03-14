;; Auto-generated. Do not edit!


(when (boundp 'mia_hand_msgs::ComponentStatus)
  (if (not (find-package "MIA_HAND_MSGS"))
    (make-package "MIA_HAND_MSGS"))
  (shadow 'ComponentStatus (find-package "MIA_HAND_MSGS")))
(unless (find-package "MIA_HAND_MSGS::COMPONENTSTATUS")
  (make-package "MIA_HAND_MSGS::COMPONENTSTATUS"))

(in-package "ROS")
;;//! \htmlinclude ComponentStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mia_hand_msgs::ComponentStatus
  :super ros::object
  :slots (_header _status _msg ))

(defmethod mia_hand_msgs::ComponentStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:status __status) nil)
    ((:msg __msg) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _status __status)
   (setq _msg (string __msg))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _status
    1
    ;; string _msg
    4 (length _msg)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get mia_hand_msgs::ComponentStatus :md5sum-) "365c676b4aefb93cfc7b974237335cea")
(setf (get mia_hand_msgs::ComponentStatus :datatype-) "mia_hand_msgs/ComponentStatus")
(setf (get mia_hand_msgs::ComponentStatus :definition-)
      "std_msgs/Header header
bool status             # True means component is ok
string msg              # Error message (when state is set to False)

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



(provide :mia_hand_msgs/ComponentStatus "365c676b4aefb93cfc7b974237335cea")


