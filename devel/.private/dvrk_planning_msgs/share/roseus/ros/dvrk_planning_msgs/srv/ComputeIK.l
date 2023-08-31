;; Auto-generated. Do not edit!


(when (boundp 'dvrk_planning_msgs::ComputeIK)
  (if (not (find-package "DVRK_PLANNING_MSGS"))
    (make-package "DVRK_PLANNING_MSGS"))
  (shadow 'ComputeIK (find-package "DVRK_PLANNING_MSGS")))
(unless (find-package "DVRK_PLANNING_MSGS::COMPUTEIK")
  (make-package "DVRK_PLANNING_MSGS::COMPUTEIK"))
(unless (find-package "DVRK_PLANNING_MSGS::COMPUTEIKREQUEST")
  (make-package "DVRK_PLANNING_MSGS::COMPUTEIKREQUEST"))
(unless (find-package "DVRK_PLANNING_MSGS::COMPUTEIKRESPONSE")
  (make-package "DVRK_PLANNING_MSGS::COMPUTEIKRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass dvrk_planning_msgs::ComputeIKRequest
  :super ros::object
  :slots (_tf_stamped ))

(defmethod dvrk_planning_msgs::ComputeIKRequest
  (:init
   (&key
    ((:tf_stamped __tf_stamped) (instance geometry_msgs::TransformStamped :init))
    )
   (send-super :init)
   (setq _tf_stamped __tf_stamped)
   self)
  (:tf_stamped
   (&rest __tf_stamped)
   (if (keywordp (car __tf_stamped))
       (send* _tf_stamped __tf_stamped)
     (progn
       (if __tf_stamped (setq _tf_stamped (car __tf_stamped)))
       _tf_stamped)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/TransformStamped _tf_stamped
    (send _tf_stamped :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/TransformStamped _tf_stamped
       (send _tf_stamped :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/TransformStamped _tf_stamped
     (send _tf_stamped :deserialize buf ptr-) (incf ptr- (send _tf_stamped :serialization-length))
   ;;
   self)
  )

(defclass dvrk_planning_msgs::ComputeIKResponse
  :super ros::object
  :slots (_joint_state ))

(defmethod dvrk_planning_msgs::ComputeIKResponse
  (:init
   (&key
    ((:joint_state __joint_state) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _joint_state __joint_state)
   self)
  (:joint_state
   (&rest __joint_state)
   (if (keywordp (car __joint_state))
       (send* _joint_state __joint_state)
     (progn
       (if __joint_state (setq _joint_state (car __joint_state)))
       _joint_state)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _joint_state
    (send _joint_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _joint_state
       (send _joint_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _joint_state
     (send _joint_state :deserialize buf ptr-) (incf ptr- (send _joint_state :serialization-length))
   ;;
   self)
  )

(defclass dvrk_planning_msgs::ComputeIK
  :super ros::object
  :slots ())

(setf (get dvrk_planning_msgs::ComputeIK :md5sum-) "ae38b9864e967b724fbb0102d7f44916")
(setf (get dvrk_planning_msgs::ComputeIK :datatype-) "dvrk_planning_msgs/ComputeIK")
(setf (get dvrk_planning_msgs::ComputeIK :request) dvrk_planning_msgs::ComputeIKRequest)
(setf (get dvrk_planning_msgs::ComputeIK :response) dvrk_planning_msgs::ComputeIKResponse)

(defmethod dvrk_planning_msgs::ComputeIKRequest
  (:response () (instance dvrk_planning_msgs::ComputeIKResponse :init)))

(setf (get dvrk_planning_msgs::ComputeIKRequest :md5sum-) "ae38b9864e967b724fbb0102d7f44916")
(setf (get dvrk_planning_msgs::ComputeIKRequest :datatype-) "dvrk_planning_msgs/ComputeIKRequest")
(setf (get dvrk_planning_msgs::ComputeIKRequest :definition-)
      "geometry_msgs/TransformStamped tf_stamped


================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

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
---

sensor_msgs/JointState joint_state


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

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
")

(setf (get dvrk_planning_msgs::ComputeIKResponse :md5sum-) "ae38b9864e967b724fbb0102d7f44916")
(setf (get dvrk_planning_msgs::ComputeIKResponse :datatype-) "dvrk_planning_msgs/ComputeIKResponse")
(setf (get dvrk_planning_msgs::ComputeIKResponse :definition-)
      "geometry_msgs/TransformStamped tf_stamped


================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

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
---

sensor_msgs/JointState joint_state


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

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
")



(provide :dvrk_planning_msgs/ComputeIK "ae38b9864e967b724fbb0102d7f44916")


