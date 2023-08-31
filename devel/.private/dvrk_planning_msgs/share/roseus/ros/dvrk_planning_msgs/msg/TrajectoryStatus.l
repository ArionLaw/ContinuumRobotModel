;; Auto-generated. Do not edit!


(when (boundp 'dvrk_planning_msgs::TrajectoryStatus)
  (if (not (find-package "DVRK_PLANNING_MSGS"))
    (make-package "DVRK_PLANNING_MSGS"))
  (shadow 'TrajectoryStatus (find-package "DVRK_PLANNING_MSGS")))
(unless (find-package "DVRK_PLANNING_MSGS::TRAJECTORYSTATUS")
  (make-package "DVRK_PLANNING_MSGS::TRAJECTORYSTATUS"))

(in-package "ROS")
;;//! \htmlinclude TrajectoryStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dvrk_planning_msgs::TrajectoryStatus
  :super ros::object
  :slots (_header _trajectory_name _percentage_completed _completed_trajectory_points _total_trajectory_points ))

(defmethod dvrk_planning_msgs::TrajectoryStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:trajectory_name __trajectory_name) "")
    ((:percentage_completed __percentage_completed) 0.0)
    ((:completed_trajectory_points __completed_trajectory_points) 0)
    ((:total_trajectory_points __total_trajectory_points) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _trajectory_name (string __trajectory_name))
   (setq _percentage_completed (float __percentage_completed))
   (setq _completed_trajectory_points (round __completed_trajectory_points))
   (setq _total_trajectory_points (round __total_trajectory_points))
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
  (:percentage_completed
   (&optional __percentage_completed)
   (if __percentage_completed (setq _percentage_completed __percentage_completed)) _percentage_completed)
  (:completed_trajectory_points
   (&optional __completed_trajectory_points)
   (if __completed_trajectory_points (setq _completed_trajectory_points __completed_trajectory_points)) _completed_trajectory_points)
  (:total_trajectory_points
   (&optional __total_trajectory_points)
   (if __total_trajectory_points (setq _total_trajectory_points __total_trajectory_points)) _total_trajectory_points)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _trajectory_name
    4 (length _trajectory_name)
    ;; float32 _percentage_completed
    4
    ;; int64 _completed_trajectory_points
    8
    ;; int64 _total_trajectory_points
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _trajectory_name
       (write-long (length _trajectory_name) s) (princ _trajectory_name s)
     ;; float32 _percentage_completed
       (sys::poke _percentage_completed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int64 _completed_trajectory_points
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _completed_trajectory_points (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _completed_trajectory_points) (= (length (_completed_trajectory_points . bv)) 2)) ;; bignum
              (write-long (ash (elt (_completed_trajectory_points . bv) 0) 0) s)
              (write-long (ash (elt (_completed_trajectory_points . bv) 1) -1) s))
             ((and (class _completed_trajectory_points) (= (length (_completed_trajectory_points . bv)) 1)) ;; big1
              (write-long (elt (_completed_trajectory_points . bv) 0) s)
              (write-long (if (>= _completed_trajectory_points 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _completed_trajectory_points s)(write-long (if (>= _completed_trajectory_points 0) 0 #xffffffff) s)))
     ;; int64 _total_trajectory_points
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _total_trajectory_points (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _total_trajectory_points) (= (length (_total_trajectory_points . bv)) 2)) ;; bignum
              (write-long (ash (elt (_total_trajectory_points . bv) 0) 0) s)
              (write-long (ash (elt (_total_trajectory_points . bv) 1) -1) s))
             ((and (class _total_trajectory_points) (= (length (_total_trajectory_points . bv)) 1)) ;; big1
              (write-long (elt (_total_trajectory_points . bv) 0) s)
              (write-long (if (>= _total_trajectory_points 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _total_trajectory_points s)(write-long (if (>= _total_trajectory_points 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _trajectory_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _trajectory_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _percentage_completed
     (setq _percentage_completed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int64 _completed_trajectory_points
#+(or :alpha :irix6 :x86_64)
      (setf _completed_trajectory_points (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _completed_trajectory_points (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _total_trajectory_points
#+(or :alpha :irix6 :x86_64)
      (setf _total_trajectory_points (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _total_trajectory_points (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get dvrk_planning_msgs::TrajectoryStatus :md5sum-) "a4e78f278b5ab5a1f55612d6d6c10761")
(setf (get dvrk_planning_msgs::TrajectoryStatus :datatype-) "dvrk_planning_msgs/TrajectoryStatus")
(setf (get dvrk_planning_msgs::TrajectoryStatus :definition-)
      "std_msgs/Header header
string trajectory_name
float32 percentage_completed
int64 completed_trajectory_points
int64 total_trajectory_points
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



(provide :dvrk_planning_msgs/TrajectoryStatus "a4e78f278b5ab5a1f55612d6d6c10761")


