; Auto-generated. Do not edit!


(cl:in-package dvrk_planning_msgs-msg)


;//! \htmlinclude TrajectoryStatus.msg.html

(cl:defclass <TrajectoryStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (trajectory_name
    :reader trajectory_name
    :initarg :trajectory_name
    :type cl:string
    :initform "")
   (percentage_completed
    :reader percentage_completed
    :initarg :percentage_completed
    :type cl:float
    :initform 0.0)
   (completed_trajectory_points
    :reader completed_trajectory_points
    :initarg :completed_trajectory_points
    :type cl:integer
    :initform 0)
   (total_trajectory_points
    :reader total_trajectory_points
    :initarg :total_trajectory_points
    :type cl:integer
    :initform 0))
)

(cl:defclass TrajectoryStatus (<TrajectoryStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajectoryStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajectoryStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dvrk_planning_msgs-msg:<TrajectoryStatus> is deprecated: use dvrk_planning_msgs-msg:TrajectoryStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrajectoryStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvrk_planning_msgs-msg:header-val is deprecated.  Use dvrk_planning_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'trajectory_name-val :lambda-list '(m))
(cl:defmethod trajectory_name-val ((m <TrajectoryStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvrk_planning_msgs-msg:trajectory_name-val is deprecated.  Use dvrk_planning_msgs-msg:trajectory_name instead.")
  (trajectory_name m))

(cl:ensure-generic-function 'percentage_completed-val :lambda-list '(m))
(cl:defmethod percentage_completed-val ((m <TrajectoryStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvrk_planning_msgs-msg:percentage_completed-val is deprecated.  Use dvrk_planning_msgs-msg:percentage_completed instead.")
  (percentage_completed m))

(cl:ensure-generic-function 'completed_trajectory_points-val :lambda-list '(m))
(cl:defmethod completed_trajectory_points-val ((m <TrajectoryStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvrk_planning_msgs-msg:completed_trajectory_points-val is deprecated.  Use dvrk_planning_msgs-msg:completed_trajectory_points instead.")
  (completed_trajectory_points m))

(cl:ensure-generic-function 'total_trajectory_points-val :lambda-list '(m))
(cl:defmethod total_trajectory_points-val ((m <TrajectoryStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dvrk_planning_msgs-msg:total_trajectory_points-val is deprecated.  Use dvrk_planning_msgs-msg:total_trajectory_points instead.")
  (total_trajectory_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajectoryStatus>) ostream)
  "Serializes a message object of type '<TrajectoryStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'trajectory_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'trajectory_name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'percentage_completed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'completed_trajectory_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'total_trajectory_points)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajectoryStatus>) istream)
  "Deserializes a message object of type '<TrajectoryStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trajectory_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'trajectory_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'percentage_completed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'completed_trajectory_points) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'total_trajectory_points) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajectoryStatus>)))
  "Returns string type for a message object of type '<TrajectoryStatus>"
  "dvrk_planning_msgs/TrajectoryStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajectoryStatus)))
  "Returns string type for a message object of type 'TrajectoryStatus"
  "dvrk_planning_msgs/TrajectoryStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajectoryStatus>)))
  "Returns md5sum for a message object of type '<TrajectoryStatus>"
  "a4e78f278b5ab5a1f55612d6d6c10761")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajectoryStatus)))
  "Returns md5sum for a message object of type 'TrajectoryStatus"
  "a4e78f278b5ab5a1f55612d6d6c10761")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajectoryStatus>)))
  "Returns full string definition for message of type '<TrajectoryStatus>"
  (cl:format cl:nil "std_msgs/Header header~%string trajectory_name~%float32 percentage_completed~%int64 completed_trajectory_points~%int64 total_trajectory_points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajectoryStatus)))
  "Returns full string definition for message of type 'TrajectoryStatus"
  (cl:format cl:nil "std_msgs/Header header~%string trajectory_name~%float32 percentage_completed~%int64 completed_trajectory_points~%int64 total_trajectory_points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajectoryStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'trajectory_name))
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajectoryStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajectoryStatus
    (cl:cons ':header (header msg))
    (cl:cons ':trajectory_name (trajectory_name msg))
    (cl:cons ':percentage_completed (percentage_completed msg))
    (cl:cons ':completed_trajectory_points (completed_trajectory_points msg))
    (cl:cons ':total_trajectory_points (total_trajectory_points msg))
))
