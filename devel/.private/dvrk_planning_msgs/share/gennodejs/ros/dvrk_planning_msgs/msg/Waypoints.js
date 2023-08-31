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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Waypoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.trajectory_name = null;
      this.instruction_mode = null;
      this.jaw_instruction = null;
      this.waypoints = null;
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
      if (initObj.hasOwnProperty('instruction_mode')) {
        this.instruction_mode = initObj.instruction_mode
      }
      else {
        this.instruction_mode = '';
      }
      if (initObj.hasOwnProperty('jaw_instruction')) {
        this.jaw_instruction = initObj.jaw_instruction
      }
      else {
        this.jaw_instruction = '';
      }
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Waypoints
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [trajectory_name]
    bufferOffset = _serializer.string(obj.trajectory_name, buffer, bufferOffset);
    // Serialize message field [instruction_mode]
    bufferOffset = _serializer.string(obj.instruction_mode, buffer, bufferOffset);
    // Serialize message field [jaw_instruction]
    bufferOffset = _serializer.string(obj.jaw_instruction, buffer, bufferOffset);
    // Serialize message field [waypoints]
    // Serialize the length for message field [waypoints]
    bufferOffset = _serializer.uint32(obj.waypoints.length, buffer, bufferOffset);
    obj.waypoints.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Transform.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Waypoints
    let len;
    let data = new Waypoints(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [trajectory_name]
    data.trajectory_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [instruction_mode]
    data.instruction_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [jaw_instruction]
    data.jaw_instruction = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [waypoints]
    // Deserialize array length for message field [waypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.waypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.waypoints[i] = geometry_msgs.msg.Transform.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.trajectory_name);
    length += _getByteLength(object.instruction_mode);
    length += _getByteLength(object.jaw_instruction);
    length += 56 * object.waypoints.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dvrk_planning_msgs/Waypoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd73f1821b5528b13bb35cf8d880f322a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Waypoints(null);
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

    if (msg.instruction_mode !== undefined) {
      resolved.instruction_mode = msg.instruction_mode;
    }
    else {
      resolved.instruction_mode = ''
    }

    if (msg.jaw_instruction !== undefined) {
      resolved.jaw_instruction = msg.jaw_instruction;
    }
    else {
      resolved.jaw_instruction = ''
    }

    if (msg.waypoints !== undefined) {
      resolved.waypoints = new Array(msg.waypoints.length);
      for (let i = 0; i < resolved.waypoints.length; ++i) {
        resolved.waypoints[i] = geometry_msgs.msg.Transform.Resolve(msg.waypoints[i]);
      }
    }
    else {
      resolved.waypoints = []
    }

    return resolved;
    }
};

module.exports = Waypoints;
