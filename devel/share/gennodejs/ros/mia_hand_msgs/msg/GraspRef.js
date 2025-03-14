// Auto-generated. Do not edit!

// (in-package mia_hand_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GraspRef {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rest = null;
      this.pos = null;
      this.delay = null;
    }
    else {
      if (initObj.hasOwnProperty('rest')) {
        this.rest = initObj.rest
      }
      else {
        this.rest = 0;
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = 0;
      }
      if (initObj.hasOwnProperty('delay')) {
        this.delay = initObj.delay
      }
      else {
        this.delay = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspRef
    // Serialize message field [rest]
    bufferOffset = _serializer.int16(obj.rest, buffer, bufferOffset);
    // Serialize message field [pos]
    bufferOffset = _serializer.int16(obj.pos, buffer, bufferOffset);
    // Serialize message field [delay]
    bufferOffset = _serializer.int16(obj.delay, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspRef
    let len;
    let data = new GraspRef(null);
    // Deserialize message field [rest]
    data.rest = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [pos]
    data.pos = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [delay]
    data.delay = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mia_hand_msgs/GraspRef';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '09f519c8adf07285290a71d280a83d8c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 rest
    int16 pos
    int16 delay
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspRef(null);
    if (msg.rest !== undefined) {
      resolved.rest = msg.rest;
    }
    else {
      resolved.rest = 0
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = 0
    }

    if (msg.delay !== undefined) {
      resolved.delay = msg.delay;
    }
    else {
      resolved.delay = 0
    }

    return resolved;
    }
};

module.exports = GraspRef;
