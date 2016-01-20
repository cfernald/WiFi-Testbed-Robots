; Auto-generated. Do not edit!


(cl:in-package proteus3_gps_hydro-msg)


;//! \htmlinclude GPSMsg.msg.html

(cl:defclass <GPSMsg> (roslisp-msg-protocol:ros-message)
  ((time_sec
    :reader time_sec
    :initarg :time_sec
    :type cl:integer
    :initform 0)
   (time_usec
    :reader time_usec
    :initarg :time_usec
    :type cl:integer
    :initform 0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (utm_e
    :reader utm_e
    :initarg :utm_e
    :type cl:float
    :initform 0.0)
   (utm_n
    :reader utm_n
    :initarg :utm_n
    :type cl:float
    :initform 0.0)
   (quality
    :reader quality
    :initarg :quality
    :type cl:fixnum
    :initform 0)
   (num_sats
    :reader num_sats
    :initarg :num_sats
    :type cl:fixnum
    :initform 0)
   (hdop
    :reader hdop
    :initarg :hdop
    :type cl:float
    :initform 0.0)
   (vdop
    :reader vdop
    :initarg :vdop
    :type cl:float
    :initform 0.0)
   (heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0))
)

(cl:defclass GPSMsg (<GPSMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPSMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPSMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name proteus3_gps_hydro-msg:<GPSMsg> is deprecated: use proteus3_gps_hydro-msg:GPSMsg instead.")))

(cl:ensure-generic-function 'time_sec-val :lambda-list '(m))
(cl:defmethod time_sec-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:time_sec-val is deprecated.  Use proteus3_gps_hydro-msg:time_sec instead.")
  (time_sec m))

(cl:ensure-generic-function 'time_usec-val :lambda-list '(m))
(cl:defmethod time_usec-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:time_usec-val is deprecated.  Use proteus3_gps_hydro-msg:time_usec instead.")
  (time_usec m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:latitude-val is deprecated.  Use proteus3_gps_hydro-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:longitude-val is deprecated.  Use proteus3_gps_hydro-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:altitude-val is deprecated.  Use proteus3_gps_hydro-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'utm_e-val :lambda-list '(m))
(cl:defmethod utm_e-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:utm_e-val is deprecated.  Use proteus3_gps_hydro-msg:utm_e instead.")
  (utm_e m))

(cl:ensure-generic-function 'utm_n-val :lambda-list '(m))
(cl:defmethod utm_n-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:utm_n-val is deprecated.  Use proteus3_gps_hydro-msg:utm_n instead.")
  (utm_n m))

(cl:ensure-generic-function 'quality-val :lambda-list '(m))
(cl:defmethod quality-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:quality-val is deprecated.  Use proteus3_gps_hydro-msg:quality instead.")
  (quality m))

(cl:ensure-generic-function 'num_sats-val :lambda-list '(m))
(cl:defmethod num_sats-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:num_sats-val is deprecated.  Use proteus3_gps_hydro-msg:num_sats instead.")
  (num_sats m))

(cl:ensure-generic-function 'hdop-val :lambda-list '(m))
(cl:defmethod hdop-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:hdop-val is deprecated.  Use proteus3_gps_hydro-msg:hdop instead.")
  (hdop m))

(cl:ensure-generic-function 'vdop-val :lambda-list '(m))
(cl:defmethod vdop-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:vdop-val is deprecated.  Use proteus3_gps_hydro-msg:vdop instead.")
  (vdop m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <GPSMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader proteus3_gps_hydro-msg:heading-val is deprecated.  Use proteus3_gps_hydro-msg:heading instead.")
  (heading m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPSMsg>) ostream)
  "Serializes a message object of type '<GPSMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_usec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_usec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_usec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_usec)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'utm_e))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'utm_n))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quality)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_sats)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hdop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vdop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPSMsg>) istream)
  "Deserializes a message object of type '<GPSMsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time_usec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time_usec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time_usec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time_usec)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'utm_e) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'utm_n) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quality)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_sats)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hdop) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vdop) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPSMsg>)))
  "Returns string type for a message object of type '<GPSMsg>"
  "proteus3_gps_hydro/GPSMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPSMsg)))
  "Returns string type for a message object of type 'GPSMsg"
  "proteus3_gps_hydro/GPSMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPSMsg>)))
  "Returns md5sum for a message object of type '<GPSMsg>"
  "086eb9058d59812396454a204ff44662")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPSMsg)))
  "Returns md5sum for a message object of type 'GPSMsg"
  "086eb9058d59812396454a204ff44662")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPSMsg>)))
  "Returns full string definition for message of type '<GPSMsg>"
  (cl:format cl:nil "#  GPS (UTC) time in seconds and microseconds since the epoch ~%uint32 time_sec~%~%# GPS (UTC) time in seconds and microseconds since the epoch ~%uint32 time_usec~%~%# Latitude in degrees~%float32 latitude~%~%# Longitude in degrees~%float32 longitude~%~%# Altitude in meters ~%float32 altitude~%~%# UTM WGS84 coordinates, easting [m] ~%float64 utm_e~%~%# UTM WGS84 coordinates, northing [m]~%float64 utm_n~%~%# Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix, 6 = Dead Reckoning ~%uint8 quality~%~%# Number of satellites in view~%uint8 num_sats~%~%# Horizontal dilution of position (HDOP)~%float32 hdop~%~%# Vertical dilution of position (VDOP)~%float32 vdop~%~%# Tracking angle of the GPS~%float32 heading~%~%# Horizontal error in meters~%# float64 err_horz~%~%# Vertical error in meters~%# float64 err_vert~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPSMsg)))
  "Returns full string definition for message of type 'GPSMsg"
  (cl:format cl:nil "#  GPS (UTC) time in seconds and microseconds since the epoch ~%uint32 time_sec~%~%# GPS (UTC) time in seconds and microseconds since the epoch ~%uint32 time_usec~%~%# Latitude in degrees~%float32 latitude~%~%# Longitude in degrees~%float32 longitude~%~%# Altitude in meters ~%float32 altitude~%~%# UTM WGS84 coordinates, easting [m] ~%float64 utm_e~%~%# UTM WGS84 coordinates, northing [m]~%float64 utm_n~%~%# Quality of fix 0 = invalid, 1 = GPS fix, 2 = DGPS fix, 6 = Dead Reckoning ~%uint8 quality~%~%# Number of satellites in view~%uint8 num_sats~%~%# Horizontal dilution of position (HDOP)~%float32 hdop~%~%# Vertical dilution of position (VDOP)~%float32 vdop~%~%# Tracking angle of the GPS~%float32 heading~%~%# Horizontal error in meters~%# float64 err_horz~%~%# Vertical error in meters~%# float64 err_vert~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPSMsg>))
  (cl:+ 0
     4
     4
     4
     4
     4
     8
     8
     1
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPSMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'GPSMsg
    (cl:cons ':time_sec (time_sec msg))
    (cl:cons ':time_usec (time_usec msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':utm_e (utm_e msg))
    (cl:cons ':utm_n (utm_n msg))
    (cl:cons ':quality (quality msg))
    (cl:cons ':num_sats (num_sats msg))
    (cl:cons ':hdop (hdop msg))
    (cl:cons ':vdop (vdop msg))
    (cl:cons ':heading (heading msg))
))
