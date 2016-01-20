; Auto-generated. Do not edit!


(cl:in-package proteus3_compass_hydro-msg)


;//! \htmlinclude CompassMsg.msg.html

(cl:defclass <CompassMsg> (roslisp-msg-protocol:ros-message)
  ((heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0))
)

(cl:defclass CompassMsg (<CompassMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CompassMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CompassMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name proteus3_compass_hydro-msg:<CompassMsg> is deprecated: use proteus3_compass_hydro-msg:CompassMsg instead.")))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <CompassMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_compass_hydro-msg:heading-val is deprecated.  Use proteus3_compass_hydro-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <CompassMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_compass_hydro-msg:pitch-val is deprecated.  Use proteus3_compass_hydro-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <CompassMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_compass_hydro-msg:roll-val is deprecated.  Use proteus3_compass_hydro-msg:roll instead.")
  (roll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CompassMsg>) ostream)
  "Serializes a message object of type '<CompassMsg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CompassMsg>) istream)
  "Deserializes a message object of type '<CompassMsg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CompassMsg>)))
  "Returns string type for a message object of type '<CompassMsg>"
  "proteus3_compass_hydro/CompassMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CompassMsg)))
  "Returns string type for a message object of type 'CompassMsg"
  "proteus3_compass_hydro/CompassMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CompassMsg>)))
  "Returns md5sum for a message object of type '<CompassMsg>"
  "a0a24f94640b168577ac5c59871cb550")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CompassMsg)))
  "Returns md5sum for a message object of type 'CompassMsg"
  "a0a24f94640b168577ac5c59871cb550")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CompassMsg>)))
  "Returns full string definition for message of type '<CompassMsg>"
  (cl:format cl:nil "float32 heading~%float32 pitch~%float32 roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CompassMsg)))
  "Returns full string definition for message of type 'CompassMsg"
  (cl:format cl:nil "float32 heading~%float32 pitch~%float32 roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CompassMsg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CompassMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'CompassMsg
    (cl:cons ':heading (heading msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
))
