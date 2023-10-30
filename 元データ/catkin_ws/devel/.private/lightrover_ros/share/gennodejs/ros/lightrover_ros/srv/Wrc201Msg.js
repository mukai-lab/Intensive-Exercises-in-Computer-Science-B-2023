// Auto-generated. Do not edit!

// (in-package lightrover_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Wrc201MsgRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.addr = null;
      this.data = null;
      this.length = null;
      this.cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('addr')) {
        this.addr = initObj.addr
      }
      else {
        this.addr = 0;
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = 0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0;
      }
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Wrc201MsgRequest
    // Serialize message field [addr]
    bufferOffset = _serializer.int8(obj.addr, buffer, bufferOffset);
    // Serialize message field [data]
    bufferOffset = _serializer.int64(obj.data, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.int8(obj.length, buffer, bufferOffset);
    // Serialize message field [cmd]
    bufferOffset = _serializer.string(obj.cmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Wrc201MsgRequest
    let len;
    let data = new Wrc201MsgRequest(null);
    // Deserialize message field [addr]
    data.addr = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [data]
    data.data = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.cmd);
    return length + 14;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lightrover_ros/Wrc201MsgRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d13a3877d7b7aa9e3c745a3c6e744d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 addr
    int64 data
    int8 length
    string cmd
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Wrc201MsgRequest(null);
    if (msg.addr !== undefined) {
      resolved.addr = msg.addr;
    }
    else {
      resolved.addr = 0
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = 0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0
    }

    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = ''
    }

    return resolved;
    }
};

class Wrc201MsgResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.readData = null;
    }
    else {
      if (initObj.hasOwnProperty('readData')) {
        this.readData = initObj.readData
      }
      else {
        this.readData = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Wrc201MsgResponse
    // Serialize message field [readData]
    bufferOffset = _serializer.int64(obj.readData, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Wrc201MsgResponse
    let len;
    let data = new Wrc201MsgResponse(null);
    // Deserialize message field [readData]
    data.readData = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'lightrover_ros/Wrc201MsgResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a9bb19b99dd5cd1f25961fbb1821b83';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 readData
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Wrc201MsgResponse(null);
    if (msg.readData !== undefined) {
      resolved.readData = msg.readData;
    }
    else {
      resolved.readData = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: Wrc201MsgRequest,
  Response: Wrc201MsgResponse,
  md5sum() { return '43791c45179089218511643bd58fdb58'; },
  datatype() { return 'lightrover_ros/Wrc201Msg'; }
};
