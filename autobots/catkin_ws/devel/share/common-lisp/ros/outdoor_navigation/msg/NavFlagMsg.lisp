; Auto-generated. Do not edit!


(cl:in-package outdoor_navigation-msg)


;//! \htmlinclude NavFlagMsg.msg.html

(cl:defclass <NavFlagMsg> (roslisp-msg-protocol:ros-message)
  ((robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:integer
    :initform 0))
)

(cl:defclass NavFlagMsg (<NavFlagMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavFlagMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavFlagMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name outdoor_navigation-msg:<NavFlagMsg> is deprecated: use outdoor_navigation-msg:NavFlagMsg instead.")))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <NavFlagMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader outdoor_navigation-msg:robot_id-val is deprecated.  Use outdoor_navigation-msg:robot_id instead.")
  (robot_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavFlagMsg>) ostream)
  "Serializes a message object of type '<NavFlagMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavFlagMsg>) istream)
  "Deserializes a message object of type '<NavFlagMsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavFlagMsg>)))
  "Returns string type for a message object of type '<NavFlagMsg>"
  "outdoor_navigation/NavFlagMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavFlagMsg)))
  "Returns string type for a message object of type 'NavFlagMsg"
  "outdoor_navigation/NavFlagMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavFlagMsg>)))
  "Returns md5sum for a message object of type '<NavFlagMsg>"
  "643c2c9b3dd26758833f013bbe8b0a36")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavFlagMsg)))
  "Returns md5sum for a message object of type 'NavFlagMsg"
  "643c2c9b3dd26758833f013bbe8b0a36")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavFlagMsg>)))
  "Returns full string definition for message of type '<NavFlagMsg>"
  (cl:format cl:nil "uint32  robot_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavFlagMsg)))
  "Returns full string definition for message of type 'NavFlagMsg"
  (cl:format cl:nil "uint32  robot_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavFlagMsg>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavFlagMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'NavFlagMsg
    (cl:cons ':robot_id (robot_id msg))
))
