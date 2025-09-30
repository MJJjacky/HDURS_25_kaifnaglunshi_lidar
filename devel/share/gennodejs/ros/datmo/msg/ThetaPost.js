// Auto-generated. Do not edit!

// (in-package datmo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ThetaPost {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta = null;
      this.distance = null;
    }
    else {
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = [];
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ThetaPost
    // Serialize message field [theta]
    bufferOffset = _arraySerializer.float32(obj.theta, buffer, bufferOffset, null);
    // Serialize message field [distance]
    bufferOffset = _arraySerializer.float32(obj.distance, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ThetaPost
    let len;
    let data = new ThetaPost(null);
    // Deserialize message field [theta]
    data.theta = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [distance]
    data.distance = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.theta.length;
    length += 4 * object.distance.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'datmo/ThetaPost';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4c4ad3bc73961ea47fc04a581b96a79c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] theta
    float32[] distance
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ThetaPost(null);
    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = []
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = []
    }

    return resolved;
    }
};

module.exports = ThetaPost;
