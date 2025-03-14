;; Auto-generated. Do not edit!


(when (boundp 'mia_hand_msgs::FingersStrainGauges)
  (if (not (find-package "MIA_HAND_MSGS"))
    (make-package "MIA_HAND_MSGS"))
  (shadow 'FingersStrainGauges (find-package "MIA_HAND_MSGS")))
(unless (find-package "MIA_HAND_MSGS::FINGERSSTRAINGAUGES")
  (make-package "MIA_HAND_MSGS::FINGERSSTRAINGAUGES"))

(in-package "ROS")
;;//! \htmlinclude FingersStrainGauges.msg.html


(defclass mia_hand_msgs::FingersStrainGauges
  :super ros::object
  :slots (_thu _ind _mrl ))

(defmethod mia_hand_msgs::FingersStrainGauges
  (:init
   (&key
    ((:thu __thu) (make-array 2 :initial-element 0 :element-type :integer))
    ((:ind __ind) (make-array 2 :initial-element 0 :element-type :integer))
    ((:mrl __mrl) (make-array 2 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _thu __thu)
   (setq _ind __ind)
   (setq _mrl __mrl)
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
    ;; int16[2] _thu
    (* 2    2)
    ;; int16[2] _ind
    (* 2    2)
    ;; int16[2] _mrl
    (* 2    2)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16[2] _thu
     (dotimes (i 2)
       (write-word (elt _thu i) s)
       )
     ;; int16[2] _ind
     (dotimes (i 2)
       (write-word (elt _ind i) s)
       )
     ;; int16[2] _mrl
     (dotimes (i 2)
       (write-word (elt _mrl i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16[2] _thu
   (dotimes (i (length _thu))
     (setf (elt _thu i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[2] _ind
   (dotimes (i (length _ind))
     (setf (elt _ind i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[2] _mrl
   (dotimes (i (length _mrl))
     (setf (elt _mrl i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;;
   self)
  )

(setf (get mia_hand_msgs::FingersStrainGauges :md5sum-) "8e0dffdebc67e94f504a570700ccd19d")
(setf (get mia_hand_msgs::FingersStrainGauges :datatype-) "mia_hand_msgs/FingersStrainGauges")
(setf (get mia_hand_msgs::FingersStrainGauges :definition-)
      "int16[2] thu
int16[2] ind
int16[2] mrl

")



(provide :mia_hand_msgs/FingersStrainGauges "8e0dffdebc67e94f504a570700ccd19d")


