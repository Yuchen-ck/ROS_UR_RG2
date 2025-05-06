;; Auto-generated. Do not edit!


(when (boundp 'pick_place_msgs::PickStep)
  (if (not (find-package "PICK_PLACE_MSGS"))
    (make-package "PICK_PLACE_MSGS"))
  (shadow 'PickStep (find-package "PICK_PLACE_MSGS")))
(unless (find-package "PICK_PLACE_MSGS::PICKSTEP")
  (make-package "PICK_PLACE_MSGS::PICKSTEP"))
(unless (find-package "PICK_PLACE_MSGS::PICKSTEPREQUEST")
  (make-package "PICK_PLACE_MSGS::PICKSTEPREQUEST"))
(unless (find-package "PICK_PLACE_MSGS::PICKSTEPRESPONSE")
  (make-package "PICK_PLACE_MSGS::PICKSTEPRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pick_place_msgs::PickStepRequest
  :super ros::object
  :slots (_step_name ))

(defmethod pick_place_msgs::PickStepRequest
  (:init
   (&key
    ((:step_name __step_name) "")
    )
   (send-super :init)
   (setq _step_name (string __step_name))
   self)
  (:step_name
   (&optional __step_name)
   (if __step_name (setq _step_name __step_name)) _step_name)
  (:serialization-length
   ()
   (+
    ;; string _step_name
    4 (length _step_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _step_name
       (write-long (length _step_name) s) (princ _step_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _step_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _step_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass pick_place_msgs::PickStepResponse
  :super ros::object
  :slots (_success _message _pose ))

(defmethod pick_place_msgs::PickStepResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _pose __pose)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass pick_place_msgs::PickStep
  :super ros::object
  :slots ())

(setf (get pick_place_msgs::PickStep :md5sum-) "efb821ff9d5a3f3a02dcfb089da0dd4e")
(setf (get pick_place_msgs::PickStep :datatype-) "pick_place_msgs/PickStep")
(setf (get pick_place_msgs::PickStep :request) pick_place_msgs::PickStepRequest)
(setf (get pick_place_msgs::PickStep :response) pick_place_msgs::PickStepResponse)

(defmethod pick_place_msgs::PickStepRequest
  (:response () (instance pick_place_msgs::PickStepResponse :init)))

(setf (get pick_place_msgs::PickStepRequest :md5sum-) "efb821ff9d5a3f3a02dcfb089da0dd4e")
(setf (get pick_place_msgs::PickStepRequest :datatype-) "pick_place_msgs/PickStepRequest")
(setf (get pick_place_msgs::PickStepRequest :definition-)
      "# -------- Request --------
string step_name
---
# -------- Response -------
bool success
string message
geometry_msgs/Pose pose


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get pick_place_msgs::PickStepResponse :md5sum-) "efb821ff9d5a3f3a02dcfb089da0dd4e")
(setf (get pick_place_msgs::PickStepResponse :datatype-) "pick_place_msgs/PickStepResponse")
(setf (get pick_place_msgs::PickStepResponse :definition-)
      "# -------- Request --------
string step_name
---
# -------- Response -------
bool success
string message
geometry_msgs/Pose pose


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :pick_place_msgs/PickStep "efb821ff9d5a3f3a02dcfb089da0dd4e")


