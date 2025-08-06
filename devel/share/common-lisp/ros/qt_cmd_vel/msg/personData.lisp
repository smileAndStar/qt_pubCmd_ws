; Auto-generated. Do not edit!


(cl:in-package qt_cmd_vel-msg)


;//! \htmlinclude personData.msg.html

(cl:defclass <personData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type cl:fixnum
    :initform 0)
   (acdata
    :reader acdata
    :initarg :acdata
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 64 :element-type 'cl:fixnum :initial-element 0))
   (heartrate
    :reader heartrate
    :initarg :heartrate
    :type cl:fixnum
    :initform 0)
   (spo2
    :reader spo2
    :initarg :spo2
    :type cl:fixnum
    :initform 0)
   (bk
    :reader bk
    :initarg :bk
    :type cl:fixnum
    :initform 0)
   (rsv1
    :reader rsv1
    :initarg :rsv1
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 8 :element-type 'cl:fixnum :initial-element 0))
   (sdnn
    :reader sdnn
    :initarg :sdnn
    :type cl:fixnum
    :initform 0)
   (rmssd
    :reader rmssd
    :initarg :rmssd
    :type cl:fixnum
    :initform 0)
   (nn50
    :reader nn50
    :initarg :nn50
    :type cl:fixnum
    :initform 0)
   (pnn50
    :reader pnn50
    :initarg :pnn50
    :type cl:fixnum
    :initform 0)
   (rra
    :reader rra
    :initarg :rra
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 6 :element-type 'cl:fixnum :initial-element 0))
   (rsv2
    :reader rsv2
    :initarg :rsv2
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass personData (<personData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <personData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'personData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qt_cmd_vel-msg:<personData> is deprecated: use qt_cmd_vel-msg:personData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:header-val is deprecated.  Use qt_cmd_vel-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'acdata-val :lambda-list '(m))
(cl:defmethod acdata-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:acdata-val is deprecated.  Use qt_cmd_vel-msg:acdata instead.")
  (acdata m))

(cl:ensure-generic-function 'heartrate-val :lambda-list '(m))
(cl:defmethod heartrate-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:heartrate-val is deprecated.  Use qt_cmd_vel-msg:heartrate instead.")
  (heartrate m))

(cl:ensure-generic-function 'spo2-val :lambda-list '(m))
(cl:defmethod spo2-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:spo2-val is deprecated.  Use qt_cmd_vel-msg:spo2 instead.")
  (spo2 m))

(cl:ensure-generic-function 'bk-val :lambda-list '(m))
(cl:defmethod bk-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:bk-val is deprecated.  Use qt_cmd_vel-msg:bk instead.")
  (bk m))

(cl:ensure-generic-function 'rsv1-val :lambda-list '(m))
(cl:defmethod rsv1-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:rsv1-val is deprecated.  Use qt_cmd_vel-msg:rsv1 instead.")
  (rsv1 m))

(cl:ensure-generic-function 'sdnn-val :lambda-list '(m))
(cl:defmethod sdnn-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:sdnn-val is deprecated.  Use qt_cmd_vel-msg:sdnn instead.")
  (sdnn m))

(cl:ensure-generic-function 'rmssd-val :lambda-list '(m))
(cl:defmethod rmssd-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:rmssd-val is deprecated.  Use qt_cmd_vel-msg:rmssd instead.")
  (rmssd m))

(cl:ensure-generic-function 'nn50-val :lambda-list '(m))
(cl:defmethod nn50-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:nn50-val is deprecated.  Use qt_cmd_vel-msg:nn50 instead.")
  (nn50 m))

(cl:ensure-generic-function 'pnn50-val :lambda-list '(m))
(cl:defmethod pnn50-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:pnn50-val is deprecated.  Use qt_cmd_vel-msg:pnn50 instead.")
  (pnn50 m))

(cl:ensure-generic-function 'rra-val :lambda-list '(m))
(cl:defmethod rra-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:rra-val is deprecated.  Use qt_cmd_vel-msg:rra instead.")
  (rra m))

(cl:ensure-generic-function 'rsv2-val :lambda-list '(m))
(cl:defmethod rsv2-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:rsv2-val is deprecated.  Use qt_cmd_vel-msg:rsv2 instead.")
  (rsv2 m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <personData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qt_cmd_vel-msg:state-val is deprecated.  Use qt_cmd_vel-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <personData>) ostream)
  "Serializes a message object of type '<personData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'header)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'acdata))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'heartrate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'spo2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bk)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'rsv1))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sdnn)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rmssd)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nn50)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pnn50)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'rra))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rsv2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <personData>) istream)
  "Deserializes a message object of type '<personData>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'header)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'acdata) (cl:make-array 64))
  (cl:let ((vals (cl:slot-value msg 'acdata)))
    (cl:dotimes (i 64)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'heartrate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'spo2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bk)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rsv1) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'rsv1)))
    (cl:dotimes (i 8)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sdnn)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rmssd)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'nn50)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pnn50)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rra) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'rra)))
    (cl:dotimes (i 6)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rsv2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<personData>)))
  "Returns string type for a message object of type '<personData>"
  "qt_cmd_vel/personData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'personData)))
  "Returns string type for a message object of type 'personData"
  "qt_cmd_vel/personData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<personData>)))
  "Returns md5sum for a message object of type '<personData>"
  "0b9dfbf73ccdfbf3d5b1a87f99434849")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'personData)))
  "Returns md5sum for a message object of type 'personData"
  "0b9dfbf73ccdfbf3d5b1a87f99434849")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<personData>)))
  "Returns full string definition for message of type '<personData>"
  (cl:format cl:nil "uint8 header        # 0xFF数据包头~%int8[64] acdata     # 心律波形数据 (有符号)~%uint8 heartrate     # 心率~%uint8 spo2          # 血氧~%uint8 bk            # 微循环~%uint8[8] rsv1       # 保留数据 1~%uint8 sdnn          # 心率变异性~%uint8 rmssd~%uint8 nn50~%uint8 pnn50~%uint8[6] rra        # RR间期~%uint8 rsv2          # 保留数据 2~%uint8 state         # 模块状态~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'personData)))
  "Returns full string definition for message of type 'personData"
  (cl:format cl:nil "uint8 header        # 0xFF数据包头~%int8[64] acdata     # 心律波形数据 (有符号)~%uint8 heartrate     # 心率~%uint8 spo2          # 血氧~%uint8 bk            # 微循环~%uint8[8] rsv1       # 保留数据 1~%uint8 sdnn          # 心率变异性~%uint8 rmssd~%uint8 nn50~%uint8 pnn50~%uint8[6] rra        # RR间期~%uint8 rsv2          # 保留数据 2~%uint8 state         # 模块状态~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <personData>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'acdata) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'rsv1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     1
     1
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'rra) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <personData>))
  "Converts a ROS message object to a list"
  (cl:list 'personData
    (cl:cons ':header (header msg))
    (cl:cons ':acdata (acdata msg))
    (cl:cons ':heartrate (heartrate msg))
    (cl:cons ':spo2 (spo2 msg))
    (cl:cons ':bk (bk msg))
    (cl:cons ':rsv1 (rsv1 msg))
    (cl:cons ':sdnn (sdnn msg))
    (cl:cons ':rmssd (rmssd msg))
    (cl:cons ':nn50 (nn50 msg))
    (cl:cons ':pnn50 (pnn50 msg))
    (cl:cons ':rra (rra msg))
    (cl:cons ':rsv2 (rsv2 msg))
    (cl:cons ':state (state msg))
))
