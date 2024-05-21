// Auto-generated. Do not edit!

// (in-package offboardholy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PTStates {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.PT_states = null;
      this.ref_states = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('PT_states')) {
        this.PT_states = initObj.PT_states
      }
      else {
        this.PT_states = new Array(10).fill(0);
      }
      if (initObj.hasOwnProperty('ref_states')) {
        this.ref_states = initObj.ref_states
      }
      else {
        this.ref_states = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PTStates
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [PT_states] has the right length
    if (obj.PT_states.length !== 10) {
      throw new Error('Unable to serialize array field PT_states - length must be 10')
    }
    // Serialize message field [PT_states]
    bufferOffset = _arraySerializer.float64(obj.PT_states, buffer, bufferOffset, 10);
    // Check that the constant length array field [ref_states] has the right length
    if (obj.ref_states.length !== 4) {
      throw new Error('Unable to serialize array field ref_states - length must be 4')
    }
    // Serialize message field [ref_states]
    bufferOffset = _arraySerializer.float64(obj.ref_states, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PTStates
    let len;
    let data = new PTStates(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [PT_states]
    data.PT_states = _arrayDeserializer.float64(buffer, bufferOffset, 10)
    // Deserialize message field [ref_states]
    data.ref_states = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'offboardholy/PTStates';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e59142698ebb422e69c6931ba7839194';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #sls_state
    std_msgs/Header header
    float64[10] PT_states
    float64[4] ref_states
    
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
    const resolved = new PTStates(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.PT_states !== undefined) {
      resolved.PT_states = msg.PT_states;
    }
    else {
      resolved.PT_states = new Array(10).fill(0)
    }

    if (msg.ref_states !== undefined) {
      resolved.ref_states = msg.ref_states;
    }
    else {
      resolved.ref_states = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = PTStates;
