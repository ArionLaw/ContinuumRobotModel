; Auto-generated. Do not edit!


(cl:in-package dvrk_planning_msgs-srv)


;//! \htmlinclude ComputeIK-request.msg.html

(cl:defclass <ComputeIK-request> (roslisp-msg-protocol:ros-message)
  ((tf_stamped
    :reader tf_stamped
    :initarg :tf_stamped
    :type geometry_msgs-msg:TransformStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TransformStamped)))
)

(cl:defclass ComputeIK-request (<ComputeIK-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeIK-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeIK-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dvrk_planning_msgs-srv:<ComputeIK-request> is deprecated: use dvrk_planning_msgs-srv:ComputeIK-request instead.")))

(cl:ensure-generic-function 'tf_stamped-val :lambda-list '(m))
(cl:defmethod tf_stamped-val ((m <ComputeIK-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvrk_planning_msgs-srv:tf_stamped-val is deprecated.  Use dvrk_planning_msgs-srv:tf_stamped instead.")
  (tf_stamped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeIK-request>) ostream)
  "Serializes a message object of type '<ComputeIK-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tf_stamped) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeIK-request>) istream)
  "Deserializes a message object of type '<ComputeIK-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tf_stamped) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeIK-request>)))
  "Returns string type for a service object of type '<ComputeIK-request>"
  "dvrk_planning_msgs/ComputeIKRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeIK-request)))
  "Returns string type for a service object of type 'ComputeIK-request"
  "dvrk_planning_msgs/ComputeIKRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeIK-request>)))
  "Returns md5sum for a message object of type '<ComputeIK-request>"
  "ae38b9864e967b724fbb0102d7f44916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeIK-request)))
  "Returns md5sum for a message object of type 'ComputeIK-request"
  "ae38b9864e967b724fbb0102d7f44916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeIK-request>)))
  "Returns full string definition for message of type '<ComputeIK-request>"
  (cl:format cl:nil "geometry_msgs/TransformStamped tf_stamped~%~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeIK-request)))
  "Returns full string definition for message of type 'ComputeIK-request"
  (cl:format cl:nil "geometry_msgs/TransformStamped tf_stamped~%~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://wiki.ros.org/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeIK-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tf_stamped))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeIK-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeIK-request
    (cl:cons ':tf_stamped (tf_stamped msg))
))
;//! \htmlinclude ComputeIK-response.msg.html

(cl:defclass <ComputeIK-response> (roslisp-msg-protocol:ros-message)
  ((joint_state
    :reader joint_state
    :initarg :joint_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass ComputeIK-response (<ComputeIK-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeIK-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeIK-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dvrk_planning_msgs-srv:<ComputeIK-response> is deprecated: use dvrk_planning_msgs-srv:ComputeIK-response instead.")))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <ComputeIK-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvrk_planning_msgs-srv:joint_state-val is deprecated.  Use dvrk_planning_msgs-srv:joint_state instead.")
  (joint_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeIK-response>) ostream)
  "Serializes a message object of type '<ComputeIK-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeIK-response>) istream)
  "Deserializes a message object of type '<ComputeIK-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeIK-response>)))
  "Returns string type for a service object of type '<ComputeIK-response>"
  "dvrk_planning_msgs/ComputeIKResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeIK-response)))
  "Returns string type for a service object of type 'ComputeIK-response"
  "dvrk_planning_msgs/ComputeIKResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeIK-response>)))
  "Returns md5sum for a message object of type '<ComputeIK-response>"
  "ae38b9864e967b724fbb0102d7f44916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeIK-response)))
  "Returns md5sum for a message object of type 'ComputeIK-response"
  "ae38b9864e967b724fbb0102d7f44916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeIK-response>)))
  "Returns full string definition for message of type '<ComputeIK-response>"
  (cl:format cl:nil "~%sensor_msgs/JointState joint_state~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeIK-response)))
  "Returns full string definition for message of type 'ComputeIK-response"
  (cl:format cl:nil "~%sensor_msgs/JointState joint_state~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeIK-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeIK-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeIK-response
    (cl:cons ':joint_state (joint_state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ComputeIK)))
  'ComputeIK-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ComputeIK)))
  'ComputeIK-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeIK)))
  "Returns string type for a service object of type '<ComputeIK>"
  "dvrk_planning_msgs/ComputeIK")