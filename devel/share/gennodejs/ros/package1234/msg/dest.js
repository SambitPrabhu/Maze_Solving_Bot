// Auto-generated. Do not edit!

// (in-package package1234.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class dest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dest_x_coordinate = null;
      this.dest_y_coordinate = null;
    }
    else {
      if (initObj.hasOwnProperty('dest_x_coordinate')) {
        this.dest_x_coordinate = initObj.dest_x_coordinate
      }
      else {
        this.dest_x_coordinate = 0.0;
      }
      if (initObj.hasOwnProperty('dest_y_coordinate')) {
        this.dest_y_coordinate = initObj.dest_y_coordinate
      }
      else {
        this.dest_y_coordinate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dest
    // Serialize message field [dest_x_coordinate]
    bufferOffset = _serializer.float64(obj.dest_x_coordinate, buffer, bufferOffset);
    // Serialize message field [dest_y_coordinate]
    bufferOffset = _serializer.float64(obj.dest_y_coordinate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dest
    let len;
    let data = new dest(null);
    // Deserialize message field [dest_x_coordinate]
    data.dest_x_coordinate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dest_y_coordinate]
    data.dest_y_coordinate = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'package1234/dest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf363d99a20f66b06e0d4259cb1930ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 dest_x_coordinate
    float64 dest_y_coordinate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new dest(null);
    if (msg.dest_x_coordinate !== undefined) {
      resolved.dest_x_coordinate = msg.dest_x_coordinate;
    }
    else {
      resolved.dest_x_coordinate = 0.0
    }

    if (msg.dest_y_coordinate !== undefined) {
      resolved.dest_y_coordinate = msg.dest_y_coordinate;
    }
    else {
      resolved.dest_y_coordinate = 0.0
    }

    return resolved;
    }
};

module.exports = dest;
