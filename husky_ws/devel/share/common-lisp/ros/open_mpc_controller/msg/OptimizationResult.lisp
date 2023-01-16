; Auto-generated. Do not edit!


(cl:in-package open_mpc_controller-msg)


;//! \htmlinclude OptimizationResult.msg.html

(cl:defclass <OptimizationResult> (roslisp-msg-protocol:ros-message)
  ((solution
    :reader solution
    :initarg :solution
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (inner_iterations
    :reader inner_iterations
    :initarg :inner_iterations
    :type cl:fixnum
    :initform 0)
   (outer_iterations
    :reader outer_iterations
    :initarg :outer_iterations
    :type cl:fixnum
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (cost
    :reader cost
    :initarg :cost
    :type cl:float
    :initform 0.0)
   (norm_fpr
    :reader norm_fpr
    :initarg :norm_fpr
    :type cl:float
    :initform 0.0)
   (penalty
    :reader penalty
    :initarg :penalty
    :type cl:float
    :initform 0.0)
   (lagrange_multipliers
    :reader lagrange_multipliers
    :initarg :lagrange_multipliers
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (infeasibility_f1
    :reader infeasibility_f1
    :initarg :infeasibility_f1
    :type cl:float
    :initform 0.0)
   (infeasibility_f2
    :reader infeasibility_f2
    :initarg :infeasibility_f2
    :type cl:float
    :initform 0.0)
   (solve_time_ms
    :reader solve_time_ms
    :initarg :solve_time_ms
    :type cl:float
    :initform 0.0))
)

(cl:defclass OptimizationResult (<OptimizationResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OptimizationResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OptimizationResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name open_mpc_controller-msg:<OptimizationResult> is deprecated: use open_mpc_controller-msg:OptimizationResult instead.")))

(cl:ensure-generic-function 'solution-val :lambda-list '(m))
(cl:defmethod solution-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:solution-val is deprecated.  Use open_mpc_controller-msg:solution instead.")
  (solution m))

(cl:ensure-generic-function 'inner_iterations-val :lambda-list '(m))
(cl:defmethod inner_iterations-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:inner_iterations-val is deprecated.  Use open_mpc_controller-msg:inner_iterations instead.")
  (inner_iterations m))

(cl:ensure-generic-function 'outer_iterations-val :lambda-list '(m))
(cl:defmethod outer_iterations-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:outer_iterations-val is deprecated.  Use open_mpc_controller-msg:outer_iterations instead.")
  (outer_iterations m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:status-val is deprecated.  Use open_mpc_controller-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'cost-val :lambda-list '(m))
(cl:defmethod cost-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:cost-val is deprecated.  Use open_mpc_controller-msg:cost instead.")
  (cost m))

(cl:ensure-generic-function 'norm_fpr-val :lambda-list '(m))
(cl:defmethod norm_fpr-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:norm_fpr-val is deprecated.  Use open_mpc_controller-msg:norm_fpr instead.")
  (norm_fpr m))

(cl:ensure-generic-function 'penalty-val :lambda-list '(m))
(cl:defmethod penalty-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:penalty-val is deprecated.  Use open_mpc_controller-msg:penalty instead.")
  (penalty m))

(cl:ensure-generic-function 'lagrange_multipliers-val :lambda-list '(m))
(cl:defmethod lagrange_multipliers-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:lagrange_multipliers-val is deprecated.  Use open_mpc_controller-msg:lagrange_multipliers instead.")
  (lagrange_multipliers m))

(cl:ensure-generic-function 'infeasibility_f1-val :lambda-list '(m))
(cl:defmethod infeasibility_f1-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:infeasibility_f1-val is deprecated.  Use open_mpc_controller-msg:infeasibility_f1 instead.")
  (infeasibility_f1 m))

(cl:ensure-generic-function 'infeasibility_f2-val :lambda-list '(m))
(cl:defmethod infeasibility_f2-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:infeasibility_f2-val is deprecated.  Use open_mpc_controller-msg:infeasibility_f2 instead.")
  (infeasibility_f2 m))

(cl:ensure-generic-function 'solve_time_ms-val :lambda-list '(m))
(cl:defmethod solve_time_ms-val ((m <OptimizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader open_mpc_controller-msg:solve_time_ms-val is deprecated.  Use open_mpc_controller-msg:solve_time_ms instead.")
  (solve_time_ms m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<OptimizationResult>)))
    "Constants for message type '<OptimizationResult>"
  '((:STATUS_CONVERGED . 0)
    (:STATUS_NOT_CONVERGED_ITERATIONS . 1)
    (:STATUS_NOT_CONVERGED_OUT_OF_TIME . 2)
    (:STATUS_NOT_CONVERGED_COST . 3)
    (:STATUS_NOT_CONVERGED_FINITE_COMPUTATION . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'OptimizationResult)))
    "Constants for message type 'OptimizationResult"
  '((:STATUS_CONVERGED . 0)
    (:STATUS_NOT_CONVERGED_ITERATIONS . 1)
    (:STATUS_NOT_CONVERGED_OUT_OF_TIME . 2)
    (:STATUS_NOT_CONVERGED_COST . 3)
    (:STATUS_NOT_CONVERGED_FINITE_COMPUTATION . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OptimizationResult>) ostream)
  "Serializes a message object of type '<OptimizationResult>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'solution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'solution))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'inner_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outer_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'outer_iterations)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'norm_fpr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'penalty))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lagrange_multipliers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lagrange_multipliers))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'infeasibility_f1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'infeasibility_f2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'solve_time_ms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OptimizationResult>) istream)
  "Deserializes a message object of type '<OptimizationResult>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'solution) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'solution)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'inner_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'outer_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'outer_iterations)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cost) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'norm_fpr) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'penalty) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lagrange_multipliers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lagrange_multipliers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infeasibility_f1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'infeasibility_f2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'solve_time_ms) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OptimizationResult>)))
  "Returns string type for a message object of type '<OptimizationResult>"
  "open_mpc_controller/OptimizationResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OptimizationResult)))
  "Returns string type for a message object of type 'OptimizationResult"
  "open_mpc_controller/OptimizationResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OptimizationResult>)))
  "Returns md5sum for a message object of type '<OptimizationResult>"
  "af1bd1e7f40ea424386b21d5312bdc7f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OptimizationResult)))
  "Returns md5sum for a message object of type 'OptimizationResult"
  "af1bd1e7f40ea424386b21d5312bdc7f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OptimizationResult>)))
  "Returns full string definition for message of type '<OptimizationResult>"
  (cl:format cl:nil "# Constants match the enumeration of status codes~%uint8 STATUS_CONVERGED=0~%uint8 STATUS_NOT_CONVERGED_ITERATIONS=1~%uint8 STATUS_NOT_CONVERGED_OUT_OF_TIME=2~%uint8 STATUS_NOT_CONVERGED_COST=3~%uint8 STATUS_NOT_CONVERGED_FINITE_COMPUTATION=4~%~%float64[]    solution              # optimizer (solution)~%uint8        inner_iterations      # number of inner iterations~%uint16       outer_iterations      # number of outer iterations~%uint8        status                # status code~%float64      cost                  # cost at solution~%float64      norm_fpr              # norm of FPR of last inner problem~%float64      penalty               # penalty value~%float64[]    lagrange_multipliers  # vector of Lagrange multipliers~%float64      infeasibility_f1      # infeasibility wrt F1~%float64      infeasibility_f2      # infeasibility wrt F2~%float64      solve_time_ms         # solution time in ms~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OptimizationResult)))
  "Returns full string definition for message of type 'OptimizationResult"
  (cl:format cl:nil "# Constants match the enumeration of status codes~%uint8 STATUS_CONVERGED=0~%uint8 STATUS_NOT_CONVERGED_ITERATIONS=1~%uint8 STATUS_NOT_CONVERGED_OUT_OF_TIME=2~%uint8 STATUS_NOT_CONVERGED_COST=3~%uint8 STATUS_NOT_CONVERGED_FINITE_COMPUTATION=4~%~%float64[]    solution              # optimizer (solution)~%uint8        inner_iterations      # number of inner iterations~%uint16       outer_iterations      # number of outer iterations~%uint8        status                # status code~%float64      cost                  # cost at solution~%float64      norm_fpr              # norm of FPR of last inner problem~%float64      penalty               # penalty value~%float64[]    lagrange_multipliers  # vector of Lagrange multipliers~%float64      infeasibility_f1      # infeasibility wrt F1~%float64      infeasibility_f2      # infeasibility wrt F2~%float64      solve_time_ms         # solution time in ms~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OptimizationResult>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'solution) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     2
     1
     8
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lagrange_multipliers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OptimizationResult>))
  "Converts a ROS message object to a list"
  (cl:list 'OptimizationResult
    (cl:cons ':solution (solution msg))
    (cl:cons ':inner_iterations (inner_iterations msg))
    (cl:cons ':outer_iterations (outer_iterations msg))
    (cl:cons ':status (status msg))
    (cl:cons ':cost (cost msg))
    (cl:cons ':norm_fpr (norm_fpr msg))
    (cl:cons ':penalty (penalty msg))
    (cl:cons ':lagrange_multipliers (lagrange_multipliers msg))
    (cl:cons ':infeasibility_f1 (infeasibility_f1 msg))
    (cl:cons ':infeasibility_f2 (infeasibility_f2 msg))
    (cl:cons ':solve_time_ms (solve_time_ms msg))
))
