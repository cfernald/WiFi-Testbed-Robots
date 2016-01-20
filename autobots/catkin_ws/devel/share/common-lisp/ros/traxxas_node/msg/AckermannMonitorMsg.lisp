; Auto-generated. Do not edit!


(cl:in-package traxxas_node-msg)


;//! \htmlinclude AckermannMonitorMsg.msg.html

(cl:defclass <AckermannMonitorMsg> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass AckermannMonitorMsg (<AckermannMonitorMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AckermannMonitorMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AckermannMonitorMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traxxas_node-msg:<AckermannMonitorMsg> is deprecated: use traxxas_node-msg:AckermannMonitorMsg instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <AckermannMonitorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_node-msg:speed-val is deprecated.  Use traxxas_node-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <AckermannMonitorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_node-msg:angle-val is deprecated.  Use traxxas_node-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AckermannMonitorMsg>) ostream)
  "Serializes a message object of type '<AckermannMonitorMsg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AckermannMonitorMsg>) istream)
  "Deserializes a message object of type '<AckermannMonitorMsg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AckermannMonitorMsg>)))
  "Returns string type for a message object of type '<AckermannMonitorMsg>"
  "traxxas_node/AckermannMonitorMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AckermannMonitorMsg)))
  "Returns string type for a message object of type 'AckermannMonitorMsg"
  "traxxas_node/AckermannMonitorMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AckermannMonitorMsg>)))
  "Returns md5sum for a message object of type '<AckermannMonitorMsg>"
  "e18a4dfdb52fee48fc2e3bc9e7b74071")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AckermannMonitorMsg)))
  "Returns md5sum for a message object of type 'AckermannMonitorMsg"
  "e18a4dfdb52fee48fc2e3bc9e7b74071")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AckermannMonitorMsg>)))
  "Returns full string definition for message of type '<AckermannMonitorMsg>"
  (cl:format cl:nil "float32 speed~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AckermannMonitorMsg)))
  "Returns full string definition for message of type 'AckermannMonitorMsg"
  (cl:format cl:nil "float32 speed~%float32 angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AckermannMonitorMsg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AckermannMonitorMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'AckermannMonitorMsg
    (cl:cons ':speed (speed msg))
    (cl:cons ':angle (angle msg))
))
