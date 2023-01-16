;; Auto-generated. Do not edit!


(when (boundp 'open_mpc_controller::OptimizationResult)
  (if (not (find-package "OPEN_MPC_CONTROLLER"))
    (make-package "OPEN_MPC_CONTROLLER"))
  (shadow 'OptimizationResult (find-package "OPEN_MPC_CONTROLLER")))
(unless (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT")
  (make-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude OptimizationResult.msg.html


(intern "*STATUS_CONVERGED*" (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(shadow '*STATUS_CONVERGED* (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(defconstant open_mpc_controller::OptimizationResult::*STATUS_CONVERGED* 0)
(intern "*STATUS_NOT_CONVERGED_ITERATIONS*" (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(shadow '*STATUS_NOT_CONVERGED_ITERATIONS* (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(defconstant open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_ITERATIONS* 1)
(intern "*STATUS_NOT_CONVERGED_OUT_OF_TIME*" (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(shadow '*STATUS_NOT_CONVERGED_OUT_OF_TIME* (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(defconstant open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_OUT_OF_TIME* 2)
(intern "*STATUS_NOT_CONVERGED_COST*" (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(shadow '*STATUS_NOT_CONVERGED_COST* (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(defconstant open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_COST* 3)
(intern "*STATUS_NOT_CONVERGED_FINITE_COMPUTATION*" (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(shadow '*STATUS_NOT_CONVERGED_FINITE_COMPUTATION* (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONRESULT"))
(defconstant open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_FINITE_COMPUTATION* 4)

(defun open_mpc_controller::OptimizationResult-to-symbol (const)
  (cond
        ((= const 0) 'open_mpc_controller::OptimizationResult::*STATUS_CONVERGED*)
        ((= const 1) 'open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_ITERATIONS*)
        ((= const 2) 'open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_OUT_OF_TIME*)
        ((= const 3) 'open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_COST*)
        ((= const 4) 'open_mpc_controller::OptimizationResult::*STATUS_NOT_CONVERGED_FINITE_COMPUTATION*)
        (t nil)))

(defclass open_mpc_controller::OptimizationResult
  :super ros::object
  :slots (_solution _inner_iterations _outer_iterations _status _cost _norm_fpr _penalty _lagrange_multipliers _infeasibility_f1 _infeasibility_f2 _solve_time_ms ))

(defmethod open_mpc_controller::OptimizationResult
  (:init
   (&key
    ((:solution __solution) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:inner_iterations __inner_iterations) 0)
    ((:outer_iterations __outer_iterations) 0)
    ((:status __status) 0)
    ((:cost __cost) 0.0)
    ((:norm_fpr __norm_fpr) 0.0)
    ((:penalty __penalty) 0.0)
    ((:lagrange_multipliers __lagrange_multipliers) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:infeasibility_f1 __infeasibility_f1) 0.0)
    ((:infeasibility_f2 __infeasibility_f2) 0.0)
    ((:solve_time_ms __solve_time_ms) 0.0)
    )
   (send-super :init)
   (setq _solution __solution)
   (setq _inner_iterations (round __inner_iterations))
   (setq _outer_iterations (round __outer_iterations))
   (setq _status (round __status))
   (setq _cost (float __cost))
   (setq _norm_fpr (float __norm_fpr))
   (setq _penalty (float __penalty))
   (setq _lagrange_multipliers __lagrange_multipliers)
   (setq _infeasibility_f1 (float __infeasibility_f1))
   (setq _infeasibility_f2 (float __infeasibility_f2))
   (setq _solve_time_ms (float __solve_time_ms))
   self)
  (:solution
   (&optional __solution)
   (if __solution (setq _solution __solution)) _solution)
  (:inner_iterations
   (&optional __inner_iterations)
   (if __inner_iterations (setq _inner_iterations __inner_iterations)) _inner_iterations)
  (:outer_iterations
   (&optional __outer_iterations)
   (if __outer_iterations (setq _outer_iterations __outer_iterations)) _outer_iterations)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:cost
   (&optional __cost)
   (if __cost (setq _cost __cost)) _cost)
  (:norm_fpr
   (&optional __norm_fpr)
   (if __norm_fpr (setq _norm_fpr __norm_fpr)) _norm_fpr)
  (:penalty
   (&optional __penalty)
   (if __penalty (setq _penalty __penalty)) _penalty)
  (:lagrange_multipliers
   (&optional __lagrange_multipliers)
   (if __lagrange_multipliers (setq _lagrange_multipliers __lagrange_multipliers)) _lagrange_multipliers)
  (:infeasibility_f1
   (&optional __infeasibility_f1)
   (if __infeasibility_f1 (setq _infeasibility_f1 __infeasibility_f1)) _infeasibility_f1)
  (:infeasibility_f2
   (&optional __infeasibility_f2)
   (if __infeasibility_f2 (setq _infeasibility_f2 __infeasibility_f2)) _infeasibility_f2)
  (:solve_time_ms
   (&optional __solve_time_ms)
   (if __solve_time_ms (setq _solve_time_ms __solve_time_ms)) _solve_time_ms)
  (:serialization-length
   ()
   (+
    ;; float64[] _solution
    (* 8    (length _solution)) 4
    ;; uint8 _inner_iterations
    1
    ;; uint16 _outer_iterations
    2
    ;; uint8 _status
    1
    ;; float64 _cost
    8
    ;; float64 _norm_fpr
    8
    ;; float64 _penalty
    8
    ;; float64[] _lagrange_multipliers
    (* 8    (length _lagrange_multipliers)) 4
    ;; float64 _infeasibility_f1
    8
    ;; float64 _infeasibility_f2
    8
    ;; float64 _solve_time_ms
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _solution
     (write-long (length _solution) s)
     (dotimes (i (length _solution))
       (sys::poke (elt _solution i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _inner_iterations
       (write-byte _inner_iterations s)
     ;; uint16 _outer_iterations
       (write-word _outer_iterations s)
     ;; uint8 _status
       (write-byte _status s)
     ;; float64 _cost
       (sys::poke _cost (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _norm_fpr
       (sys::poke _norm_fpr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _penalty
       (sys::poke _penalty (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _lagrange_multipliers
     (write-long (length _lagrange_multipliers) s)
     (dotimes (i (length _lagrange_multipliers))
       (sys::poke (elt _lagrange_multipliers i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _infeasibility_f1
       (sys::poke _infeasibility_f1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _infeasibility_f2
       (sys::poke _infeasibility_f2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _solve_time_ms
       (sys::poke _solve_time_ms (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _solution
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _solution (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _solution i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; uint8 _inner_iterations
     (setq _inner_iterations (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _outer_iterations
     (setq _outer_iterations (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _cost
     (setq _cost (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _norm_fpr
     (setq _norm_fpr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _penalty
     (setq _penalty (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _lagrange_multipliers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lagrange_multipliers (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _lagrange_multipliers i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _infeasibility_f1
     (setq _infeasibility_f1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _infeasibility_f2
     (setq _infeasibility_f2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _solve_time_ms
     (setq _solve_time_ms (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get open_mpc_controller::OptimizationResult :md5sum-) "af1bd1e7f40ea424386b21d5312bdc7f")
(setf (get open_mpc_controller::OptimizationResult :datatype-) "open_mpc_controller/OptimizationResult")
(setf (get open_mpc_controller::OptimizationResult :definition-)
      "# Constants match the enumeration of status codes
uint8 STATUS_CONVERGED=0
uint8 STATUS_NOT_CONVERGED_ITERATIONS=1
uint8 STATUS_NOT_CONVERGED_OUT_OF_TIME=2
uint8 STATUS_NOT_CONVERGED_COST=3
uint8 STATUS_NOT_CONVERGED_FINITE_COMPUTATION=4

float64[]    solution              # optimizer (solution)
uint8        inner_iterations      # number of inner iterations
uint16       outer_iterations      # number of outer iterations
uint8        status                # status code
float64      cost                  # cost at solution
float64      norm_fpr              # norm of FPR of last inner problem
float64      penalty               # penalty value
float64[]    lagrange_multipliers  # vector of Lagrange multipliers
float64      infeasibility_f1      # infeasibility wrt F1
float64      infeasibility_f2      # infeasibility wrt F2
float64      solve_time_ms         # solution time in ms


")



(provide :open_mpc_controller/OptimizationResult "af1bd1e7f40ea424386b21d5312bdc7f")


