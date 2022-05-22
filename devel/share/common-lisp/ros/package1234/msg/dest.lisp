; Auto-generated. Do not edit!


(cl:in-package package1234-msg)


;//! \htmlinclude dest.msg.html

(cl:defclass <dest> (roslisp-msg-protocol:ros-message)
  ((dest_x_coordinate
    :reader dest_x_coordinate
    :initarg :dest_x_coordinate
    :type cl:float
    :initform 0.0)
   (dest_y_coordinate
    :reader dest_y_coordinate
    :initarg :dest_y_coordinate
    :type cl:float
    :initform 0.0))
)

(cl:defclass dest (<dest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name package1234-msg:<dest> is deprecated: use package1234-msg:dest instead.")))

(cl:ensure-generic-function 'dest_x_coordinate-val :lambda-list '(m))
(cl:defmethod dest_x_coordinate-val ((m <dest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package1234-msg:dest_x_coordinate-val is deprecated.  Use package1234-msg:dest_x_coordinate instead.")
  (dest_x_coordinate m))

(cl:ensure-generic-function 'dest_y_coordinate-val :lambda-list '(m))
(cl:defmethod dest_y_coordinate-val ((m <dest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader package1234-msg:dest_y_coordinate-val is deprecated.  Use package1234-msg:dest_y_coordinate instead.")
  (dest_y_coordinate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dest>) ostream)
  "Serializes a message object of type '<dest>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dest_x_coordinate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dest_y_coordinate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dest>) istream)
  "Deserializes a message object of type '<dest>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dest_x_coordinate) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dest_y_coordinate) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dest>)))
  "Returns string type for a message object of type '<dest>"
  "package1234/dest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dest)))
  "Returns string type for a message object of type 'dest"
  "package1234/dest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dest>)))
  "Returns md5sum for a message object of type '<dest>"
  "cf363d99a20f66b06e0d4259cb1930ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dest)))
  "Returns md5sum for a message object of type 'dest"
  "cf363d99a20f66b06e0d4259cb1930ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dest>)))
  "Returns full string definition for message of type '<dest>"
  (cl:format cl:nil "float64 dest_x_coordinate~%float64 dest_y_coordinate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dest)))
  "Returns full string definition for message of type 'dest"
  (cl:format cl:nil "float64 dest_x_coordinate~%float64 dest_y_coordinate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dest>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dest>))
  "Converts a ROS message object to a list"
  (cl:list 'dest
    (cl:cons ':dest_x_coordinate (dest_x_coordinate msg))
    (cl:cons ':dest_y_coordinate (dest_y_coordinate msg))
))
