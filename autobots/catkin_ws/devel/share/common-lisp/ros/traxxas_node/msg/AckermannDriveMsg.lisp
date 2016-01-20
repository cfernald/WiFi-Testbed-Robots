; Auto-generated. Do not edit!


(cl:in-package traxxas_node-msg)


;//! \htmlinclude AckermannDriveMsg.msg.html

(cl:defclass <AckermannDriveMsg> (roslisp-msg-protocol:ros-message)
  ((steering_angle
    :reader steering_angle
    :initarg :steering_angle
    :type cl:float
    :initform 0.0)
   (steering_angle_velocity
    :reader steering_angle_velocity
    :initarg :steering_angle_velocity
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0)
   (jerk
    :reader jerk
    :initarg :jerk
    :type cl:float
    :initform 0.0))
)

(cl:defclass AckermannDriveMsg (<AckermannDriveMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AckermannDriveMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AckermannDriveMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traxxas_node-msg:<AckermannDriveMsg> is deprecated: use traxxas_node-msg:AckermannDriveMsg instead.")))

(cl:ensure-generic-function 'steering_angle-val :lambda-list '(m))
(cl:defmethod steering_angle-val ((m <AckermannDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_node-msg:steering_angle-val is deprecated.  Use traxxas_node-msg:steering_angle instead.")
  (steering_angle m))

(cl:ensure-generic-function 'steering_angle_velocity-val :lambda-list '(m))
(cl:defmethod steering_angle_velocity-val ((m <AckermannDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_node-msg:steering_angle_velocity-val is deprecated.  Use traxxas_node-msg:steering_angle_velocity instead.")
  (steering_angle_velocity m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <AckermannDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_node-msg:speed-val is deprecated.  Use traxxas_node-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <AckermannDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_node-msg:acceleration-val is deprecated.  Use traxxas_node-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'jerk-val :lambda-list '(m))
(cl:defmethod jerk-val ((m <AckermannDriveMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traxxas_node-msg:jerk-val is deprecated.  Use traxxas_node-msg:jerk instead.")
  (jerk m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AckermannDriveMsg>) ostream)
  "Serializes a message object of type '<AckermannDriveMsg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steering_angle_velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'jerk))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AckermannDriveMsg>) istream)
  "Deserializes a message object of type '<AckermannDriveMsg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steering_angle_velocity) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'jerk) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AckermannDriveMsg>)))
  "Returns string type for a message object of type '<AckermannDriveMsg>"
  "traxxas_node/AckermannDriveMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AckermannDriveMsg)))
  "Returns string type for a message object of type 'AckermannDriveMsg"
  "traxxas_node/AckermannDriveMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AckermannDriveMsg>)))
  "Returns md5sum for a message object of type '<AckermannDriveMsg>"
  "3512e91b48d69674a0e86fadf1ea8231")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AckermannDriveMsg)))
  "Returns md5sum for a message object of type 'AckermannDriveMsg"
  "3512e91b48d69674a0e86fadf1ea8231")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AckermannDriveMsg>)))
  "Returns full string definition for message of type '<AckermannDriveMsg>"
  (cl:format cl:nil "# Average angle of the front wheels in radians (left is positive).~%# This angle is the true turning angle for bicycles and motorcycles. ~%# It is an approximation for a car, computed by taking the average ~%# of the angles of the 2 front wheels.~%float32 steering_angle~%~%# Desired rate of change of the steering angle in radians/second.~%float32 steering_angle_velocity~%~%# Speed of the center of the rear axle in meters/second (reverse is negative)~%float32 speed~%~%# Desired acceleration of the center of the rear axle in meters/second^2~%float32 acceleration~%~%# Desired jerk of the center of the rear axle in meters/second^3~%float32 jerk~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AckermannDriveMsg)))
  "Returns full string definition for message of type 'AckermannDriveMsg"
  (cl:format cl:nil "# Average angle of the front wheels in radians (left is positive).~%# This angle is the true turning angle for bicycles and motorcycles. ~%# It is an approximation for a car, computed by taking the average ~%# of the angles of the 2 front wheels.~%float32 steering_angle~%~%# Desired rate of change of the steering angle in radians/second.~%float32 steering_angle_velocity~%~%# Speed of the center of the rear axle in meters/second (reverse is negative)~%float32 speed~%~%# Desired acceleration of the center of the rear axle in meters/second^2~%float32 acceleration~%~%# Desired jerk of the center of the rear axle in meters/second^3~%float32 jerk~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AckermannDriveMsg>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AckermannDriveMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'AckermannDriveMsg
    (cl:cons ':steering_angle (steering_angle msg))
    (cl:cons ':steering_angle_velocity (steering_angle_velocity msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':jerk (jerk msg))
))
