// Auto-generated. Do not edit!

// (in-package lightrover_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class color_detector {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.color_type = null;
    }
    else {
      if (initObj.hasOwnProperty('color_type')) {
        this.color_type = initObj.color_type
      }
      else {
        this.color_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type color_detector
    // Serialize message field [color_type]
    bufferOffset = _serializer.int8(obj.color_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type color_detector
    let len;
    let data = new color_detector(null);
    // Deserialize message field [color_type]
    data.color_type = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lightrover_ros/color_detector';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28ff8e773f1e211eaff66787abb1695e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 color_type
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new color_detector(null);
    if (msg.color_type !== undefined) {
      resolved.color_type = msg.color_type;
    }
    else {
      resolved.color_type = 0
    }

    return resolved;
    }
};

module.exports = color_detector;
