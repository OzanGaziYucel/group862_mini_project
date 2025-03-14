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

class FingersStrainGauges {
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
        this.thu = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('ind')) {
        this.ind = initObj.ind
      }
      else {
        this.ind = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('mrl')) {
        this.mrl = initObj.mrl
      }
      else {
        this.mrl = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FingersStrainGauges
    // Check that the constant length array field [thu] has the right length
    if (obj.thu.length !== 2) {
      throw new Error('Unable to serialize array field thu - length must be 2')
    }
    // Serialize message field [thu]
    bufferOffset = _arraySerializer.int16(obj.thu, buffer, bufferOffset, 2);
    // Check that the constant length array field [ind] has the right length
    if (obj.ind.length !== 2) {
      throw new Error('Unable to serialize array field ind - length must be 2')
    }
    // Serialize message field [ind]
    bufferOffset = _arraySerializer.int16(obj.ind, buffer, bufferOffset, 2);
    // Check that the constant length array field [mrl] has the right length
    if (obj.mrl.length !== 2) {
      throw new Error('Unable to serialize array field mrl - length must be 2')
    }
    // Serialize message field [mrl]
    bufferOffset = _arraySerializer.int16(obj.mrl, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FingersStrainGauges
    let len;
    let data = new FingersStrainGauges(null);
    // Deserialize message field [thu]
    data.thu = _arrayDeserializer.int16(buffer, bufferOffset, 2)
    // Deserialize message field [ind]
    data.ind = _arrayDeserializer.int16(buffer, bufferOffset, 2)
    // Deserialize message field [mrl]
    data.mrl = _arrayDeserializer.int16(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mia_hand_msgs/FingersStrainGauges';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8e0dffdebc67e94f504a570700ccd19d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16[2] thu
    int16[2] ind
    int16[2] mrl
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FingersStrainGauges(null);
    if (msg.thu !== undefined) {
      resolved.thu = msg.thu;
    }
    else {
      resolved.thu = new Array(2).fill(0)
    }

    if (msg.ind !== undefined) {
      resolved.ind = msg.ind;
    }
    else {
      resolved.ind = new Array(2).fill(0)
    }

    if (msg.mrl !== undefined) {
      resolved.mrl = msg.mrl;
    }
    else {
      resolved.mrl = new Array(2).fill(0)
    }

    return resolved;
    }
};

module.exports = FingersStrainGauges;
