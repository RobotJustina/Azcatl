; Auto-generated. Do not edit!


(cl:in-package light_node-srv)


;//! \htmlinclude LightSrv-request.msg.html

(cl:defclass <LightSrv-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:integer
    :initform 0))
)

(cl:defclass LightSrv-request (<LightSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LightSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LightSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name light_node-srv:<LightSrv-request> is deprecated: use light_node-srv:LightSrv-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <LightSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader light_node-srv:req-val is deprecated.  Use light_node-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LightSrv-request>) ostream)
  "Serializes a message object of type '<LightSrv-request>"
  (cl:let* ((signed (cl:slot-value msg 'req)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LightSrv-request>) istream)
  "Deserializes a message object of type '<LightSrv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'req) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LightSrv-request>)))
  "Returns string type for a service object of type '<LightSrv-request>"
  "light_node/LightSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LightSrv-request)))
  "Returns string type for a service object of type 'LightSrv-request"
  "light_node/LightSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LightSrv-request>)))
  "Returns md5sum for a message object of type '<LightSrv-request>"
  "880a7d45ae2b2447f4ed19a09614d287")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LightSrv-request)))
  "Returns md5sum for a message object of type 'LightSrv-request"
  "880a7d45ae2b2447f4ed19a09614d287")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LightSrv-request>)))
  "Returns full string definition for message of type '<LightSrv-request>"
  (cl:format cl:nil "int32 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LightSrv-request)))
  "Returns full string definition for message of type 'LightSrv-request"
  (cl:format cl:nil "int32 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LightSrv-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LightSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LightSrv-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude LightSrv-response.msg.html

(cl:defclass <LightSrv-response> (roslisp-msg-protocol:ros-message)
  ((quantized_attraction
    :reader quantized_attraction
    :initarg :quantized_attraction
    :type cl:integer
    :initform 0)
   (quantized_intensity
    :reader quantized_intensity
    :initarg :quantized_intensity
    :type cl:integer
    :initform 0))
)

(cl:defclass LightSrv-response (<LightSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LightSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LightSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name light_node-srv:<LightSrv-response> is deprecated: use light_node-srv:LightSrv-response instead.")))

(cl:ensure-generic-function 'quantized_attraction-val :lambda-list '(m))
(cl:defmethod quantized_attraction-val ((m <LightSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader light_node-srv:quantized_attraction-val is deprecated.  Use light_node-srv:quantized_attraction instead.")
  (quantized_attraction m))

(cl:ensure-generic-function 'quantized_intensity-val :lambda-list '(m))
(cl:defmethod quantized_intensity-val ((m <LightSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader light_node-srv:quantized_intensity-val is deprecated.  Use light_node-srv:quantized_intensity instead.")
  (quantized_intensity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LightSrv-response>) ostream)
  "Serializes a message object of type '<LightSrv-response>"
  (cl:let* ((signed (cl:slot-value msg 'quantized_attraction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'quantized_intensity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LightSrv-response>) istream)
  "Deserializes a message object of type '<LightSrv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'quantized_attraction) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'quantized_intensity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LightSrv-response>)))
  "Returns string type for a service object of type '<LightSrv-response>"
  "light_node/LightSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LightSrv-response)))
  "Returns string type for a service object of type 'LightSrv-response"
  "light_node/LightSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LightSrv-response>)))
  "Returns md5sum for a message object of type '<LightSrv-response>"
  "880a7d45ae2b2447f4ed19a09614d287")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LightSrv-response)))
  "Returns md5sum for a message object of type 'LightSrv-response"
  "880a7d45ae2b2447f4ed19a09614d287")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LightSrv-response>)))
  "Returns full string definition for message of type '<LightSrv-response>"
  (cl:format cl:nil "int32 quantized_attraction~%int32 quantized_intensity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LightSrv-response)))
  "Returns full string definition for message of type 'LightSrv-response"
  (cl:format cl:nil "int32 quantized_attraction~%int32 quantized_intensity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LightSrv-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LightSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LightSrv-response
    (cl:cons ':quantized_attraction (quantized_attraction msg))
    (cl:cons ':quantized_intensity (quantized_intensity msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LightSrv)))
  'LightSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LightSrv)))
  'LightSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LightSrv)))
  "Returns string type for a service object of type '<LightSrv>"
  "light_node/LightSrv")