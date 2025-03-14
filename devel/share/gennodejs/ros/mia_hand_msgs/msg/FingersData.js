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

class FingersData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.thu = null;
      this.ind = null;
      this.mrl = null;
    }
    else {
      if (initObj.hasOwnProperty('thu')) {
        this.thu = initObj.thu
      }
      else {
        this.thu = 0;
      }
      if (initObj.hasOwnProperty('ind')) {
        this.ind = initObj.ind
      }
      else {
        this.ind = 0;
      }
      if (initObj.hasOwnProperty('mrl')) {
        this.mrl = initObj.mrl
      }
      else {
        this.mrl = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FingersData
    // Serialize message field [thu]
    bufferOffset = _serializer.int16(obj.thu, buffer, bufferOffset);
    // Serialize message field [ind]
    bufferOffset = _serializer.int16(obj.ind, buffer, bufferOffset);
    // Serialize message field [mrl]
    bufferOffset = _serializer.int16(obj.mrl, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FingersData
    let len;
    let data = new FingersData(null);
    // Deserialize message field [thu]
    data.thu = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [ind]
    data.ind = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [mrl]
    data.mrl = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mia_hand_msgs/FingersData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '67f7588122a2649383b5219661ab68d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 thu
    int16 ind
    int16 mrl
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FingersData(null);
    if (msg.thu !== undefined) {
      resolved.thu = msg.thu;
    }
    else {
      resolved.thu = 0
    }

    if (msg.ind !== undefined) {
      resolved.ind = msg.ind;
    }
    else {
      resolved.ind = 0
    }

    if (msg.mrl !== undefined) {
      resolved.mrl = msg.mrl;
    }
    else {
      resolved.mrl = 0
    }

    return resolved;
    }
};

module.exports = FingersData;
