; Auto-generated. Do not edit!


(cl:in-package pick_place_msgs-srv)


;//! \htmlinclude PickStep-request.msg.html

(cl:defclass <PickStep-request> (roslisp-msg-protocol:ros-message)
  ((step_name
    :reader step_name
    :initarg :step_name
    :type cl:string
    :initform ""))
)

(cl:defclass PickStep-request (<PickStep-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PickStep-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PickStep-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_place_msgs-srv:<PickStep-request> is deprecated: use pick_place_msgs-srv:PickStep-request instead.")))

(cl:ensure-generic-function 'step_name-val :lambda-list '(m))
(cl:defmethod step_name-val ((m <PickStep-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_place_msgs-srv:step_name-val is deprecated.  Use pick_place_msgs-srv:step_name instead.")
  (step_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PickStep-request>) ostream)
  "Serializes a message object of type '<PickStep-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'step_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'step_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PickStep-request>) istream)
  "Deserializes a message object of type '<PickStep-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'step_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'step_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PickStep-request>)))
  "Returns string type for a service object of type '<PickStep-request>"
  "pick_place_msgs/PickStepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickStep-request)))
  "Returns string type for a service object of type 'PickStep-request"
  "pick_place_msgs/PickStepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PickStep-request>)))
  "Returns md5sum for a message object of type '<PickStep-request>"
  "efb821ff9d5a3f3a02dcfb089da0dd4e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PickStep-request)))
  "Returns md5sum for a message object of type 'PickStep-request"
  "efb821ff9d5a3f3a02dcfb089da0dd4e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PickStep-request>)))
  "Returns full string definition for message of type '<PickStep-request>"
  (cl:format cl:nil "# -------- Request --------~%string step_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PickStep-request)))
  "Returns full string definition for message of type 'PickStep-request"
  (cl:format cl:nil "# -------- Request --------~%string step_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PickStep-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'step_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PickStep-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PickStep-request
    (cl:cons ':step_name (step_name msg))
))
;//! \htmlinclude PickStep-response.msg.html

(cl:defclass <PickStep-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass PickStep-response (<PickStep-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PickStep-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PickStep-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_place_msgs-srv:<PickStep-response> is deprecated: use pick_place_msgs-srv:PickStep-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PickStep-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_place_msgs-srv:success-val is deprecated.  Use pick_place_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <PickStep-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_place_msgs-srv:message-val is deprecated.  Use pick_place_msgs-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <PickStep-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_place_msgs-srv:pose-val is deprecated.  Use pick_place_msgs-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PickStep-response>) ostream)
  "Serializes a message object of type '<PickStep-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PickStep-response>) istream)
  "Deserializes a message object of type '<PickStep-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PickStep-response>)))
  "Returns string type for a service object of type '<PickStep-response>"
  "pick_place_msgs/PickStepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickStep-response)))
  "Returns string type for a service object of type 'PickStep-response"
  "pick_place_msgs/PickStepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PickStep-response>)))
  "Returns md5sum for a message object of type '<PickStep-response>"
  "efb821ff9d5a3f3a02dcfb089da0dd4e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PickStep-response)))
  "Returns md5sum for a message object of type 'PickStep-response"
  "efb821ff9d5a3f3a02dcfb089da0dd4e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PickStep-response>)))
  "Returns full string definition for message of type '<PickStep-response>"
  (cl:format cl:nil "# -------- Response -------~%bool success~%string message~%geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PickStep-response)))
  "Returns full string definition for message of type 'PickStep-response"
  (cl:format cl:nil "# -------- Response -------~%bool success~%string message~%geometry_msgs/Pose pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PickStep-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PickStep-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PickStep-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':pose (pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PickStep)))
  'PickStep-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PickStep)))
  'PickStep-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PickStep)))
  "Returns string type for a service object of type '<PickStep>"
  "pick_place_msgs/PickStep")