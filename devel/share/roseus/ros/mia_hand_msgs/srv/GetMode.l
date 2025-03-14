;; Auto-generated. Do not edit!


(when (boundp 'mia_hand_msgs::GetMode)
  (if (not (find-package "MIA_HAND_MSGS"))
    (make-package "MIA_HAND_MSGS"))
  (shadow 'GetMode (find-package "MIA_HAND_MSGS")))
(unless (find-package "MIA_HAND_MSGS::GETMODE")
  (make-package "MIA_HAND_MSGS::GETMODE"))
(unless (find-package "MIA_HAND_MSGS::GETMODEREQUEST")
  (make-package "MIA_HAND_MSGS::GETMODEREQUEST"))
(unless (find-package "MIA_HAND_MSGS::GETMODERESPONSE")
  (make-package "MIA_HAND_MSGS::GETMODERESPONSE"))

(in-package "ROS")





(defclass mia_hand_msgs::GetModeRequest
  :super ros::object
  :slots ())

(defmethod mia_hand_msgs::GetModeRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(intern "*NORMAL_OPERATION*" (find-package "MIA_HAND_MSGS::GETMODERESPONSE"))
(shadow '*NORMAL_OPERATION* (find-package "MIA_HAND_MSGS::GETMODERESPONSE"))
(defconstant mia_hand_msgs::GetModeResponse::*NORMAL_OPERATION* 0)
(intern "*PAUSED*" (find-package "MIA_HAND_MSGS::GETMODERESPONSE"))
(shadow '*PAUSED* (find-package "MIA_HAND_MSGS::GETMODERESPONSE"))
(defconstant mia_hand_msgs::GetModeResponse::*PAUSED* 1)

(defun mia_hand_msgs::GetModeResponse-to-symbol (const)
  (cond
        ((= const 0) 'mia_hand_msgs::GetModeResponse::*NORMAL_OPERATION*)
        ((= const 1) 'mia_hand_msgs::GetModeResponse::*PAUSED*)
        (t nil)))

(defclass mia_hand_msgs::GetModeResponse
  :super ros::object
  :slots (_mode ))

(defmethod mia_hand_msgs::GetModeResponse
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mia_hand_msgs::GetMode
  :super ros::object
  :slots ())

(setf (get mia_hand_msgs::GetMode :md5sum-) "bfda0693d1611db496d948702503f0f4")
(setf (get mia_hand_msgs::GetMode :datatype-) "mia_hand_msgs/GetMode")
(setf (get mia_hand_msgs::GetMode :request) mia_hand_msgs::GetModeRequest)
(setf (get mia_hand_msgs::GetMode :response) mia_hand_msgs::GetModeResponse)

(defmethod mia_hand_msgs::GetModeRequest
  (:response () (instance mia_hand_msgs::GetModeResponse :init)))

(setf (get mia_hand_msgs::GetModeRequest :md5sum-) "bfda0693d1611db496d948702503f0f4")
(setf (get mia_hand_msgs::GetModeRequest :datatype-) "mia_hand_msgs/GetModeRequest")
(setf (get mia_hand_msgs::GetModeRequest :definition-)
      "---
uint8 NORMAL_OPERATION = 0
uint8 PAUSED = 1

uint8 mode

")

(setf (get mia_hand_msgs::GetModeResponse :md5sum-) "bfda0693d1611db496d948702503f0f4")
(setf (get mia_hand_msgs::GetModeResponse :datatype-) "mia_hand_msgs/GetModeResponse")
(setf (get mia_hand_msgs::GetModeResponse :definition-)
      "---
uint8 NORMAL_OPERATION = 0
uint8 PAUSED = 1

uint8 mode

")



(provide :mia_hand_msgs/GetMode "bfda0693d1611db496d948702503f0f4")


