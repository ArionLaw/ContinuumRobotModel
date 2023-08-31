;; Auto-generated. Do not edit!


(when (boundp 'dvrk_planning_msgs::Waypoints)
  (if (not (find-package "DVRK_PLANNING_MSGS"))
    (make-package "DVRK_PLANNING_MSGS"))
  (shadow 'Waypoints (find-package "DVRK_PLANNING_MSGS")))
(unless (find-package "DVRK_PLANNING_MSGS::WAYPOINTS")
  (make-package "DVRK_PLANNING_MSGS::WAYPOINTS"))

(in-package "ROS")
;;//! \htmlinclude Waypoints.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dvrk_planning_msgs::Waypoints
  :super ros::object
  :slots (_header _trajectory_name _instruction_mode _jaw_instruction _waypoints ))

(defmethod dvrk_planning_msgs::Waypoints
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:trajectory_name __trajectory_name) "")
    ((:instruction_mode __instruction_mode) "")
    ((:jaw_instruction __jaw_instruction) "")
    ((:waypoints __waypoints) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _trajectory_name (string __trajectory_name))
   (setq _instruction_mode (string __instruction_mode))
   (setq _jaw_instruction (string __jaw_instruction))
   (setq _waypoints __waypoints)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:trajectory_name
   (&optional __trajectory_name)
   (if __trajectory_name (setq _trajectory_name __trajectory_name)) _trajectory_name)
  (:instruction_mode
   (&optional __instruction_mode)
   (if __instruction_mode (setq _instruction_mode __instruction_mode)) _instruction_mode)
  (:jaw_instruction
   (&optional __jaw_instruction)
   (if __jaw_instruction (setq _jaw_instruction __jaw_instruction)) _jaw_instruction)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _trajectory_name
    4 (length _trajectory_name)
    ;; string _instruction_mode
    4 (length _instruction_mode)
    ;; string _jaw_instruction
    4 (length _jaw_instruction)
    ;; geometry_msgs/Transform[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _trajectory_name
       (write-long (length _trajectory_name) s) (princ _trajectory_name s)
     ;; string _instruction_mode
       (write-long (length _instruction_mode) s) (princ _instruction_mode s)
     ;; string _jaw_instruction
       (write-long (length _jaw_instruction) s) (princ _jaw_instruction s)
     ;; geometry_msgs/Transform[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _trajectory_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _trajectory_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _instruction_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _instruction_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _jaw_instruction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _jaw_instruction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Transform[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance geometry_msgs::Transform :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dvrk_planning_msgs::Waypoints :md5sum-) "d73f1821b5528b13bb35cf8d880f322a")
(setf (get dvrk_planning_msgs::Waypoints :datatype-) "dvrk_planning_msgs/Waypoints")
(setf (get dvrk_planning_msgs::Waypoints :definition-)
      "std_msgs/Header header
string trajectory_name
string instruction_mode  
string jaw_instruction               
geometry_msgs/Transform[] waypoints
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :dvrk_planning_msgs/Waypoints "d73f1821b5528b13bb35cf8d880f322a")


