;; Auto-generated. Do not edit!


(when (boundp 'mia_hand_msgs::GraspRef)
  (if (not (find-package "MIA_HAND_MSGS"))
    (make-package "MIA_HAND_MSGS"))
  (shadow 'GraspRef (find-package "MIA_HAND_MSGS")))
(unless (find-package "MIA_HAND_MSGS::GRASPREF")
  (make-package "MIA_HAND_MSGS::GRASPREF"))

(in-package "ROS")
;;//! \htmlinclude GraspRef.msg.html


(defclass mia_hand_msgs::GraspRef
  :super ros::object
  :slots (_rest _pos _delay ))

(defmethod mia_hand_msgs::GraspRef
  (:init
   (&key
    ((:rest __rest) 0)
    ((:pos __pos) 0)
    ((:delay __delay) 0)
    )
   (send-super :init)
   (setq _rest (round __rest))
   (setq _pos (round __pos))
   (setq _delay (round __delay))
   self)
  (:rest
   (&optional __rest)
   (if __rest (setq _rest __rest)) _rest)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:delay
   (&optional __delay)
   (if __delay (setq _delay __delay)) _delay)
  (:serialization-length
   ()
   (+
    ;; int16 _rest
    2
    ;; int16 _pos
    2
    ;; int16 _delay
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _rest
       (write-word _rest s)
     ;; int16 _pos
       (write-word _pos s)
     ;; int16 _delay
       (write-word _delay s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _rest
     (setq _rest (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _pos
     (setq _pos (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _delay
     (setq _delay (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mia_hand_msgs::GraspRef :md5sum-) "09f519c8adf07285290a71d280a83d8c")
(setf (get mia_hand_msgs::GraspRef :datatype-) "mia_hand_msgs/GraspRef")
(setf (get mia_hand_msgs::GraspRef :definition-)
      "int16 rest
int16 pos
int16 delay

")



(provide :mia_hand_msgs/GraspRef "09f519c8adf07285290a71d280a83d8c")


