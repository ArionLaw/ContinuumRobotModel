// Auto-generated. Do not edit!

// (in-package dvrk_planning_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TrajectoryStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.trajectory_name = null;
      this.percentage_completed = null;
      this.completed_trajectory_points = null;
      this.total_trajectory_points = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('trajectory_name')) {
        this.trajectory_name = initObj.trajectory_name
      }
      else {
        this.trajectory_name = '';
      }
      if (initObj.hasOwnProperty('percentage_completed')) {
        this.percentage_completed = initObj.percentage_completed
      }
      else {
        this.percentage_completed = 0.0;
      }
      if (initObj.hasOwnProperty('completed_trajectory_points')) {
        this.completed_trajectory_points = initObj.completed_trajectory_points
      }
      else {
        this.completed_trajectory_points = 0;
      }
      if (initObj.hasOwnProperty('total_trajectory_points')) {
        this.total_trajectory_points = initObj.total_trajectory_points
      }
      else {
        this.total_trajectory_points = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [trajectory_name]
    bufferOffset = _serializer.string(obj.trajectory_name, buffer, bufferOffset);
    // Serialize message field [percentage_completed]
    bufferOffset = _serializer.float32(obj.percentage_completed, buffer, bufferOffset);
    // Serialize message field [completed_trajectory_points]
    bufferOffset = _serializer.int64(obj.completed_trajectory_points, buffer, bufferOffset);
    // Serialize message field [total_trajectory_points]
    bufferOffset = _serializer.int64(obj.total_trajectory_points, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryStatus
    let len;
    let data = new TrajectoryStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [trajectory_name]
    data.trajectory_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [percentage_completed]
    data.percentage_completed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [completed_trajectory_points]
    data.completed_trajectory_points = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [total_trajectory_points]
    data.total_trajectory_points = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.trajectory_name);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dvrk_planning_msgs/TrajectoryStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4e78f278b5ab5a1f55612d6d6c10761';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.trajectory_name !== undefined) {
      resolved.trajectory_name = msg.trajectory_name;
    }
    else {
      resolved.trajectory_name = ''
    }

    if (msg.percentage_completed !== undefined) {
      resolved.percentage_completed = msg.percentage_completed;
    }
    else {
      resolved.percentage_completed = 0.0
    }

    if (msg.completed_trajectory_points !== undefined) {
      resolved.completed_trajectory_points = msg.completed_trajectory_points;
    }
    else {
      resolved.completed_trajectory_points = 0
    }

    if (msg.total_trajectory_points !== undefined) {
      resolved.total_trajectory_points = msg.total_trajectory_points;
    }
    else {
      resolved.total_trajectory_points = 0
    }

    return resolved;
    }
};

module.exports = TrajectoryStatus;
