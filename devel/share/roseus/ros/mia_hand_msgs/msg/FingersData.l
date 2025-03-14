;; Auto-generated. Do not edit!


(when (boundp 'mia_hand_msgs::FingersData)
  (if (not (find-package "MIA_HAND_MSGS"))
    (make-package "MIA_HAND_MSGS"))
  (shadow 'FingersData (find-package "MIA_HAND_MSGS")))
(unless (find-package "MIA_HAND_MSGS::FINGERSDATA")
  (make-package "MIA_HAND_MSGS::FINGERSDATA"))

(in-package "ROS")
;;//! \htmlinclude FingersData.msg.html


(defclass mia_hand_msgs::FingersData
  :super ros::object
  :slots (_thu _ind _mrl ))

(defmethod mia_hand_msgs::FingersData
  (:init
   (&key
    ((:thu __thu) 0)
    ((:ind __ind) 0)
    ((:mrl __mrl) 0)
    )
   (send-super :init)
   (setq _thu (round __thu))
   (setq _ind (round __ind))
   (setq _mrl (round __mrl))
   self)
  (:thu
   (&optional __thu)
   (if __thu (setq _thu __thu)) _thu)
  (:ind
   (&optional __ind)
   (if __ind (setq _ind __ind)) _ind)
  (:mrl
   (&optional __mrl)
   (if __mrl (setq _mrl __mrl)) _mrl)
  (:serialization-length
   ()
   (+
    ;; int16 _thu
    2
    ;; int16 _ind
    2
    ;; int16 _mrl
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _thu
       (write-word _thu s)
     ;; int16 _ind
       (write-word _ind s)
     ;; int16 _mrl
       (write-word _mrl s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _thu
     (setq _thu (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _ind
     (setq _ind (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _mrl
     (setq _mrl (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mia_hand_msgs::FingersData :md5sum-) "67f7588122a2649383b5219661ab68d4")
(setf (get mia_hand_msgs::FingersData :datatype-) "mia_hand_msgs/FingersData")
(setf (get mia_hand_msgs::FingersData :definition-)
      "int16 thu
int16 ind
int16 mrl

")



(provide :mia_hand_msgs/FingersData "67f7588122a2649383b5219661ab68d4")


