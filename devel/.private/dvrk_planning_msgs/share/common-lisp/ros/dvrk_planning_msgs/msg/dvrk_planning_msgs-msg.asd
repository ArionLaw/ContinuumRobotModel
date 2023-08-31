
(cl:in-package :asdf)

(defsystem "dvrk_planning_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TrajectoryStatus" :depends-on ("_package_TrajectoryStatus"))
    (:file "_package_TrajectoryStatus" :depends-on ("_package"))
    (:file "Waypoints" :depends-on ("_package_Waypoints"))
    (:file "_package_Waypoints" :depends-on ("_package"))
  ))