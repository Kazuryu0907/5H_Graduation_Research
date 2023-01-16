;; Auto-generated. Do not edit!


(when (boundp 'open_mpc_controller::OptimizationParameters)
  (if (not (find-package "OPEN_MPC_CONTROLLER"))
    (make-package "OPEN_MPC_CONTROLLER"))
  (shadow 'OptimizationParameters (find-package "OPEN_MPC_CONTROLLER")))
(unless (find-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONPARAMETERS")
  (make-package "OPEN_MPC_CONTROLLER::OPTIMIZATIONPARAMETERS"))

(in-package "ROS")
;;//! \htmlinclude OptimizationParameters.msg.html


(defclass open_mpc_controller::OptimizationParameters
  :super ros::object
  :slots (_parameter _initial_guess _initial_y _initial_penalty ))

(defmethod open_mpc_controller::OptimizationParameters
  (:init
   (&key
    ((:parameter __parameter) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:initial_guess __initial_guess) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:initial_y __initial_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:initial_penalty __initial_penalty) 0.0)
    )
   (send-super :init)
   (setq _parameter __parameter)
   (setq _initial_guess __initial_guess)
   (setq _initial_y __initial_y)
   (setq _initial_penalty (float __initial_penalty))
   self)
  (:parameter
   (&optional __parameter)
   (if __parameter (setq _parameter __parameter)) _parameter)
  (:initial_guess
   (&optional __initial_guess)
   (if __initial_guess (setq _initial_guess __initial_guess)) _initial_guess)
  (:initial_y
   (&optional __initial_y)
   (if __initial_y (setq _initial_y __initial_y)) _initial_y)
  (:initial_penalty
   (&optional __initial_penalty)
   (if __initial_penalty (setq _initial_penalty __initial_penalty)) _initial_penalty)
  (:serialization-length
   ()
   (+
    ;; float64[] _parameter
    (* 8    (length _parameter)) 4
    ;; float64[] _initial_guess
    (* 8    (length _initial_guess)) 4
    ;; float64[] _initial_y
    (* 8    (length _initial_y)) 4
    ;; float64 _initial_penalty
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _parameter
     (write-long (length _parameter) s)
     (dotimes (i (length _parameter))
       (sys::poke (elt _parameter i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _initial_guess
     (write-long (length _initial_guess) s)
     (dotimes (i (length _initial_guess))
       (sys::poke (elt _initial_guess i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _initial_y
     (write-long (length _initial_y) s)
     (dotimes (i (length _initial_y))
       (sys::poke (elt _initial_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _initial_penalty
       (sys::poke _initial_penalty (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _parameter
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _parameter (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _parameter i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _initial_guess
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _initial_guess (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _initial_guess i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _initial_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _initial_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _initial_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _initial_penalty
     (setq _initial_penalty (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get open_mpc_controller::OptimizationParameters :md5sum-) "00f21f74662b284bc463c6f9b40a4c7f")
(setf (get open_mpc_controller::OptimizationParameters :datatype-) "open_mpc_controller/OptimizationParameters")
(setf (get open_mpc_controller::OptimizationParameters :definition-)
      "float64[]      parameter         # parameter p (mandatory)
float64[]      initial_guess     # u0 (optional/recommended)
float64[]      initial_y         # y0 (optional)
float64        initial_penalty   # initial penalty (optional)

")



(provide :open_mpc_controller/OptimizationParameters "00f21f74662b284bc463c6f9b40a4c7f")


