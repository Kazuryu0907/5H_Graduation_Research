// Auto-generated. Do not edit!

// (in-package open_mpc_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class OptimizationParameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.parameter = null;
      this.initial_guess = null;
      this.initial_y = null;
      this.initial_penalty = null;
    }
    else {
      if (initObj.hasOwnProperty('parameter')) {
        this.parameter = initObj.parameter
      }
      else {
        this.parameter = [];
      }
      if (initObj.hasOwnProperty('initial_guess')) {
        this.initial_guess = initObj.initial_guess
      }
      else {
        this.initial_guess = [];
      }
      if (initObj.hasOwnProperty('initial_y')) {
        this.initial_y = initObj.initial_y
      }
      else {
        this.initial_y = [];
      }
      if (initObj.hasOwnProperty('initial_penalty')) {
        this.initial_penalty = initObj.initial_penalty
      }
      else {
        this.initial_penalty = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OptimizationParameters
    // Serialize message field [parameter]
    bufferOffset = _arraySerializer.float64(obj.parameter, buffer, bufferOffset, null);
    // Serialize message field [initial_guess]
    bufferOffset = _arraySerializer.float64(obj.initial_guess, buffer, bufferOffset, null);
    // Serialize message field [initial_y]
    bufferOffset = _arraySerializer.float64(obj.initial_y, buffer, bufferOffset, null);
    // Serialize message field [initial_penalty]
    bufferOffset = _serializer.float64(obj.initial_penalty, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OptimizationParameters
    let len;
    let data = new OptimizationParameters(null);
    // Deserialize message field [parameter]
    data.parameter = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [initial_guess]
    data.initial_guess = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [initial_y]
    data.initial_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [initial_penalty]
    data.initial_penalty = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.parameter.length;
    length += 8 * object.initial_guess.length;
    length += 8 * object.initial_y.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'open_mpc_controller/OptimizationParameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '00f21f74662b284bc463c6f9b40a4c7f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[]      parameter         # parameter p (mandatory)
    float64[]      initial_guess     # u0 (optional/recommended)
    float64[]      initial_y         # y0 (optional)
    float64        initial_penalty   # initial penalty (optional)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OptimizationParameters(null);
    if (msg.parameter !== undefined) {
      resolved.parameter = msg.parameter;
    }
    else {
      resolved.parameter = []
    }

    if (msg.initial_guess !== undefined) {
      resolved.initial_guess = msg.initial_guess;
    }
    else {
      resolved.initial_guess = []
    }

    if (msg.initial_y !== undefined) {
      resolved.initial_y = msg.initial_y;
    }
    else {
      resolved.initial_y = []
    }

    if (msg.initial_penalty !== undefined) {
      resolved.initial_penalty = msg.initial_penalty;
    }
    else {
      resolved.initial_penalty = 0.0
    }

    return resolved;
    }
};

module.exports = OptimizationParameters;
