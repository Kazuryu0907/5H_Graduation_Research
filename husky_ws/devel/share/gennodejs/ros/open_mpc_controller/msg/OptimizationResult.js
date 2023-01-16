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

class OptimizationResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.solution = null;
      this.inner_iterations = null;
      this.outer_iterations = null;
      this.status = null;
      this.cost = null;
      this.norm_fpr = null;
      this.penalty = null;
      this.lagrange_multipliers = null;
      this.infeasibility_f1 = null;
      this.infeasibility_f2 = null;
      this.solve_time_ms = null;
    }
    else {
      if (initObj.hasOwnProperty('solution')) {
        this.solution = initObj.solution
      }
      else {
        this.solution = [];
      }
      if (initObj.hasOwnProperty('inner_iterations')) {
        this.inner_iterations = initObj.inner_iterations
      }
      else {
        this.inner_iterations = 0;
      }
      if (initObj.hasOwnProperty('outer_iterations')) {
        this.outer_iterations = initObj.outer_iterations
      }
      else {
        this.outer_iterations = 0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('cost')) {
        this.cost = initObj.cost
      }
      else {
        this.cost = 0.0;
      }
      if (initObj.hasOwnProperty('norm_fpr')) {
        this.norm_fpr = initObj.norm_fpr
      }
      else {
        this.norm_fpr = 0.0;
      }
      if (initObj.hasOwnProperty('penalty')) {
        this.penalty = initObj.penalty
      }
      else {
        this.penalty = 0.0;
      }
      if (initObj.hasOwnProperty('lagrange_multipliers')) {
        this.lagrange_multipliers = initObj.lagrange_multipliers
      }
      else {
        this.lagrange_multipliers = [];
      }
      if (initObj.hasOwnProperty('infeasibility_f1')) {
        this.infeasibility_f1 = initObj.infeasibility_f1
      }
      else {
        this.infeasibility_f1 = 0.0;
      }
      if (initObj.hasOwnProperty('infeasibility_f2')) {
        this.infeasibility_f2 = initObj.infeasibility_f2
      }
      else {
        this.infeasibility_f2 = 0.0;
      }
      if (initObj.hasOwnProperty('solve_time_ms')) {
        this.solve_time_ms = initObj.solve_time_ms
      }
      else {
        this.solve_time_ms = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OptimizationResult
    // Serialize message field [solution]
    bufferOffset = _arraySerializer.float64(obj.solution, buffer, bufferOffset, null);
    // Serialize message field [inner_iterations]
    bufferOffset = _serializer.uint8(obj.inner_iterations, buffer, bufferOffset);
    // Serialize message field [outer_iterations]
    bufferOffset = _serializer.uint16(obj.outer_iterations, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [cost]
    bufferOffset = _serializer.float64(obj.cost, buffer, bufferOffset);
    // Serialize message field [norm_fpr]
    bufferOffset = _serializer.float64(obj.norm_fpr, buffer, bufferOffset);
    // Serialize message field [penalty]
    bufferOffset = _serializer.float64(obj.penalty, buffer, bufferOffset);
    // Serialize message field [lagrange_multipliers]
    bufferOffset = _arraySerializer.float64(obj.lagrange_multipliers, buffer, bufferOffset, null);
    // Serialize message field [infeasibility_f1]
    bufferOffset = _serializer.float64(obj.infeasibility_f1, buffer, bufferOffset);
    // Serialize message field [infeasibility_f2]
    bufferOffset = _serializer.float64(obj.infeasibility_f2, buffer, bufferOffset);
    // Serialize message field [solve_time_ms]
    bufferOffset = _serializer.float64(obj.solve_time_ms, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OptimizationResult
    let len;
    let data = new OptimizationResult(null);
    // Deserialize message field [solution]
    data.solution = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [inner_iterations]
    data.inner_iterations = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [outer_iterations]
    data.outer_iterations = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [cost]
    data.cost = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [norm_fpr]
    data.norm_fpr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [penalty]
    data.penalty = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [lagrange_multipliers]
    data.lagrange_multipliers = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [infeasibility_f1]
    data.infeasibility_f1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [infeasibility_f2]
    data.infeasibility_f2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [solve_time_ms]
    data.solve_time_ms = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.solution.length;
    length += 8 * object.lagrange_multipliers.length;
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'open_mpc_controller/OptimizationResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'af1bd1e7f40ea424386b21d5312bdc7f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Constants match the enumeration of status codes
    uint8 STATUS_CONVERGED=0
    uint8 STATUS_NOT_CONVERGED_ITERATIONS=1
    uint8 STATUS_NOT_CONVERGED_OUT_OF_TIME=2
    uint8 STATUS_NOT_CONVERGED_COST=3
    uint8 STATUS_NOT_CONVERGED_FINITE_COMPUTATION=4
    
    float64[]    solution              # optimizer (solution)
    uint8        inner_iterations      # number of inner iterations
    uint16       outer_iterations      # number of outer iterations
    uint8        status                # status code
    float64      cost                  # cost at solution
    float64      norm_fpr              # norm of FPR of last inner problem
    float64      penalty               # penalty value
    float64[]    lagrange_multipliers  # vector of Lagrange multipliers
    float64      infeasibility_f1      # infeasibility wrt F1
    float64      infeasibility_f2      # infeasibility wrt F2
    float64      solve_time_ms         # solution time in ms
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OptimizationResult(null);
    if (msg.solution !== undefined) {
      resolved.solution = msg.solution;
    }
    else {
      resolved.solution = []
    }

    if (msg.inner_iterations !== undefined) {
      resolved.inner_iterations = msg.inner_iterations;
    }
    else {
      resolved.inner_iterations = 0
    }

    if (msg.outer_iterations !== undefined) {
      resolved.outer_iterations = msg.outer_iterations;
    }
    else {
      resolved.outer_iterations = 0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.cost !== undefined) {
      resolved.cost = msg.cost;
    }
    else {
      resolved.cost = 0.0
    }

    if (msg.norm_fpr !== undefined) {
      resolved.norm_fpr = msg.norm_fpr;
    }
    else {
      resolved.norm_fpr = 0.0
    }

    if (msg.penalty !== undefined) {
      resolved.penalty = msg.penalty;
    }
    else {
      resolved.penalty = 0.0
    }

    if (msg.lagrange_multipliers !== undefined) {
      resolved.lagrange_multipliers = msg.lagrange_multipliers;
    }
    else {
      resolved.lagrange_multipliers = []
    }

    if (msg.infeasibility_f1 !== undefined) {
      resolved.infeasibility_f1 = msg.infeasibility_f1;
    }
    else {
      resolved.infeasibility_f1 = 0.0
    }

    if (msg.infeasibility_f2 !== undefined) {
      resolved.infeasibility_f2 = msg.infeasibility_f2;
    }
    else {
      resolved.infeasibility_f2 = 0.0
    }

    if (msg.solve_time_ms !== undefined) {
      resolved.solve_time_ms = msg.solve_time_ms;
    }
    else {
      resolved.solve_time_ms = 0.0
    }

    return resolved;
    }
};

// Constants for message
OptimizationResult.Constants = {
  STATUS_CONVERGED: 0,
  STATUS_NOT_CONVERGED_ITERATIONS: 1,
  STATUS_NOT_CONVERGED_OUT_OF_TIME: 2,
  STATUS_NOT_CONVERGED_COST: 3,
  STATUS_NOT_CONVERGED_FINITE_COMPUTATION: 4,
}

module.exports = OptimizationResult;
