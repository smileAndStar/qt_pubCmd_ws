;; Auto-generated. Do not edit!


(when (boundp 'qt_cmd_vel::personData)
  (if (not (find-package "QT_CMD_VEL"))
    (make-package "QT_CMD_VEL"))
  (shadow 'personData (find-package "QT_CMD_VEL")))
(unless (find-package "QT_CMD_VEL::PERSONDATA")
  (make-package "QT_CMD_VEL::PERSONDATA"))

(in-package "ROS")
;;//! \htmlinclude personData.msg.html


(defclass qt_cmd_vel::personData
  :super ros::object
  :slots (_header _acdata _heartrate _spo2 _bk _rsv1 _sdnn _rmssd _nn50 _pnn50 _rra _rsv2 _state ))

(defmethod qt_cmd_vel::personData
  (:init
   (&key
    ((:header __header) 0)
    ((:acdata __acdata) (make-array 64 :initial-element 0 :element-type :integer))
    ((:heartrate __heartrate) 0)
    ((:spo2 __spo2) 0)
    ((:bk __bk) 0)
    ((:rsv1 __rsv1) (make-array 8 :initial-element 0 :element-type :char))
    ((:sdnn __sdnn) 0)
    ((:rmssd __rmssd) 0)
    ((:nn50 __nn50) 0)
    ((:pnn50 __pnn50) 0)
    ((:rra __rra) (make-array 6 :initial-element 0 :element-type :char))
    ((:rsv2 __rsv2) 0)
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _header (round __header))
   (setq _acdata __acdata)
   (setq _heartrate (round __heartrate))
   (setq _spo2 (round __spo2))
   (setq _bk (round __bk))
   (setq _rsv1 __rsv1)
   (setq _sdnn (round __sdnn))
   (setq _rmssd (round __rmssd))
   (setq _nn50 (round __nn50))
   (setq _pnn50 (round __pnn50))
   (setq _rra __rra)
   (setq _rsv2 (round __rsv2))
   (setq _state (round __state))
   self)
  (:header
   (&optional __header)
   (if __header (setq _header __header)) _header)
  (:acdata
   (&optional __acdata)
   (if __acdata (setq _acdata __acdata)) _acdata)
  (:heartrate
   (&optional __heartrate)
   (if __heartrate (setq _heartrate __heartrate)) _heartrate)
  (:spo2
   (&optional __spo2)
   (if __spo2 (setq _spo2 __spo2)) _spo2)
  (:bk
   (&optional __bk)
   (if __bk (setq _bk __bk)) _bk)
  (:rsv1
   (&optional __rsv1)
   (if __rsv1 (setq _rsv1 __rsv1)) _rsv1)
  (:sdnn
   (&optional __sdnn)
   (if __sdnn (setq _sdnn __sdnn)) _sdnn)
  (:rmssd
   (&optional __rmssd)
   (if __rmssd (setq _rmssd __rmssd)) _rmssd)
  (:nn50
   (&optional __nn50)
   (if __nn50 (setq _nn50 __nn50)) _nn50)
  (:pnn50
   (&optional __pnn50)
   (if __pnn50 (setq _pnn50 __pnn50)) _pnn50)
  (:rra
   (&optional __rra)
   (if __rra (setq _rra __rra)) _rra)
  (:rsv2
   (&optional __rsv2)
   (if __rsv2 (setq _rsv2 __rsv2)) _rsv2)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _header
    1
    ;; int8[64] _acdata
    (* 1    64)
    ;; uint8 _heartrate
    1
    ;; uint8 _spo2
    1
    ;; uint8 _bk
    1
    ;; uint8[8] _rsv1
    (* 1    8)
    ;; uint8 _sdnn
    1
    ;; uint8 _rmssd
    1
    ;; uint8 _nn50
    1
    ;; uint8 _pnn50
    1
    ;; uint8[6] _rra
    (* 1    6)
    ;; uint8 _rsv2
    1
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _header
       (write-byte _header s)
     ;; int8[64] _acdata
     (dotimes (i 64)
       (write-byte (elt _acdata i) s)
       )
     ;; uint8 _heartrate
       (write-byte _heartrate s)
     ;; uint8 _spo2
       (write-byte _spo2 s)
     ;; uint8 _bk
       (write-byte _bk s)
     ;; uint8[8] _rsv1
     (princ _rsv1 s)
     ;; uint8 _sdnn
       (write-byte _sdnn s)
     ;; uint8 _rmssd
       (write-byte _rmssd s)
     ;; uint8 _nn50
       (write-byte _nn50 s)
     ;; uint8 _pnn50
       (write-byte _pnn50 s)
     ;; uint8[6] _rra
     (princ _rra s)
     ;; uint8 _rsv2
       (write-byte _rsv2 s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _header
     (setq _header (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int8[64] _acdata
   (dotimes (i (length _acdata))
     (setf (elt _acdata i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _acdata i) 127) (setf (elt _acdata i) (- (elt _acdata i) 256)))
     )
   ;; uint8 _heartrate
     (setq _heartrate (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _spo2
     (setq _spo2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _bk
     (setq _bk (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[8] _rsv1
   (setq _rsv1 (make-array 8 :element-type :char))
   (replace _rsv1 buf :start2 ptr-) (incf ptr- 8)
   ;; uint8 _sdnn
     (setq _sdnn (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rmssd
     (setq _rmssd (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _nn50
     (setq _nn50 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _pnn50
     (setq _pnn50 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[6] _rra
   (setq _rra (make-array 6 :element-type :char))
   (replace _rra buf :start2 ptr-) (incf ptr- 6)
   ;; uint8 _rsv2
     (setq _rsv2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get qt_cmd_vel::personData :md5sum-) "0b9dfbf73ccdfbf3d5b1a87f99434849")
(setf (get qt_cmd_vel::personData :datatype-) "qt_cmd_vel/personData")
(setf (get qt_cmd_vel::personData :definition-)
      "uint8 header        # 0xFF数据包头
int8[64] acdata     # 心律波形数据 (有符号)
uint8 heartrate     # 心率
uint8 spo2          # 血氧
uint8 bk            # 微循环
uint8[8] rsv1       # 保留数据 1
uint8 sdnn          # 心率变异性
uint8 rmssd
uint8 nn50
uint8 pnn50
uint8[6] rra        # RR间期
uint8 rsv2          # 保留数据 2
uint8 state         # 模块状态
")



(provide :qt_cmd_vel/personData "0b9dfbf73ccdfbf3d5b1a87f99434849")


