;; Auto-generated. Do not edit!


(when (boundp 'mia_hand_msgs::ConnectSerial)
  (if (not (find-package "MIA_HAND_MSGS"))
    (make-package "MIA_HAND_MSGS"))
  (shadow 'ConnectSerial (find-package "MIA_HAND_MSGS")))
(unless (find-package "MIA_HAND_MSGS::CONNECTSERIAL")
  (make-package "MIA_HAND_MSGS::CONNECTSERIAL"))
(unless (find-package "MIA_HAND_MSGS::CONNECTSERIALREQUEST")
  (make-package "MIA_HAND_MSGS::CONNECTSERIALREQUEST"))
(unless (find-package "MIA_HAND_MSGS::CONNECTSERIALRESPONSE")
  (make-package "MIA_HAND_MSGS::CONNECTSERIALRESPONSE"))

(in-package "ROS")





(defclass mia_hand_msgs::ConnectSerialRequest
  :super ros::object
  :slots (_port ))

(defmethod mia_hand_msgs::ConnectSerialRequest
  (:init
   (&key
    ((:port __port) 0)
    )
   (send-super :init)
   (setq _port (round __port))
   self)
  (:port
   (&optional __port)
   (if __port (setq _port __port)) _port)
  (:serialization-length
   ()
   (+
    ;; uint16 _port
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _port
       (write-word _port s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _port
     (setq _port (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass mia_hand_msgs::ConnectSerialResponse
  :super ros::object
  :slots (_success _message ))

(defmethod mia_hand_msgs::ConnectSerialResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mia_hand_msgs::ConnectSerial
  :super ros::object
  :slots ())

(setf (get mia_hand_msgs::ConnectSerial :md5sum-) "86e89cea09faaef62a0ac1903573e480")
(setf (get mia_hand_msgs::ConnectSerial :datatype-) "mia_hand_msgs/ConnectSerial")
(setf (get mia_hand_msgs::ConnectSerial :request) mia_hand_msgs::ConnectSerialRequest)
(setf (get mia_hand_msgs::ConnectSerial :response) mia_hand_msgs::ConnectSerialResponse)

(defmethod mia_hand_msgs::ConnectSerialRequest
  (:response () (instance mia_hand_msgs::ConnectSerialResponse :init)))

(setf (get mia_hand_msgs::ConnectSerialRequest :md5sum-) "86e89cea09faaef62a0ac1903573e480")
(setf (get mia_hand_msgs::ConnectSerialRequest :datatype-) "mia_hand_msgs/ConnectSerialRequest")
(setf (get mia_hand_msgs::ConnectSerialRequest :definition-)
      "uint16 port
---
bool success
string message

")

(setf (get mia_hand_msgs::ConnectSerialResponse :md5sum-) "86e89cea09faaef62a0ac1903573e480")
(setf (get mia_hand_msgs::ConnectSerialResponse :datatype-) "mia_hand_msgs/ConnectSerialResponse")
(setf (get mia_hand_msgs::ConnectSerialResponse :definition-)
      "uint16 port
---
bool success
string message

")



(provide :mia_hand_msgs/ConnectSerial "86e89cea09faaef62a0ac1903573e480")


