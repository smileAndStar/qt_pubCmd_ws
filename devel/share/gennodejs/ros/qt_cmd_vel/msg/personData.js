// Auto-generated. Do not edit!

// (in-package qt_cmd_vel.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class personData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.acdata = null;
      this.heartrate = null;
      this.spo2 = null;
      this.bk = null;
      this.rsv1 = null;
      this.sdnn = null;
      this.rmssd = null;
      this.nn50 = null;
      this.pnn50 = null;
      this.rra = null;
      this.rsv2 = null;
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = 0;
      }
      if (initObj.hasOwnProperty('acdata')) {
        this.acdata = initObj.acdata
      }
      else {
        this.acdata = new Array(64).fill(0);
      }
      if (initObj.hasOwnProperty('heartrate')) {
        this.heartrate = initObj.heartrate
      }
      else {
        this.heartrate = 0;
      }
      if (initObj.hasOwnProperty('spo2')) {
        this.spo2 = initObj.spo2
      }
      else {
        this.spo2 = 0;
      }
      if (initObj.hasOwnProperty('bk')) {
        this.bk = initObj.bk
      }
      else {
        this.bk = 0;
      }
      if (initObj.hasOwnProperty('rsv1')) {
        this.rsv1 = initObj.rsv1
      }
      else {
        this.rsv1 = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('sdnn')) {
        this.sdnn = initObj.sdnn
      }
      else {
        this.sdnn = 0;
      }
      if (initObj.hasOwnProperty('rmssd')) {
        this.rmssd = initObj.rmssd
      }
      else {
        this.rmssd = 0;
      }
      if (initObj.hasOwnProperty('nn50')) {
        this.nn50 = initObj.nn50
      }
      else {
        this.nn50 = 0;
      }
      if (initObj.hasOwnProperty('pnn50')) {
        this.pnn50 = initObj.pnn50
      }
      else {
        this.pnn50 = 0;
      }
      if (initObj.hasOwnProperty('rra')) {
        this.rra = initObj.rra
      }
      else {
        this.rra = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('rsv2')) {
        this.rsv2 = initObj.rsv2
      }
      else {
        this.rsv2 = 0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type personData
    // Serialize message field [header]
    bufferOffset = _serializer.uint8(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [acdata] has the right length
    if (obj.acdata.length !== 64) {
      throw new Error('Unable to serialize array field acdata - length must be 64')
    }
    // Serialize message field [acdata]
    bufferOffset = _arraySerializer.int8(obj.acdata, buffer, bufferOffset, 64);
    // Serialize message field [heartrate]
    bufferOffset = _serializer.uint8(obj.heartrate, buffer, bufferOffset);
    // Serialize message field [spo2]
    bufferOffset = _serializer.uint8(obj.spo2, buffer, bufferOffset);
    // Serialize message field [bk]
    bufferOffset = _serializer.uint8(obj.bk, buffer, bufferOffset);
    // Check that the constant length array field [rsv1] has the right length
    if (obj.rsv1.length !== 8) {
      throw new Error('Unable to serialize array field rsv1 - length must be 8')
    }
    // Serialize message field [rsv1]
    bufferOffset = _arraySerializer.uint8(obj.rsv1, buffer, bufferOffset, 8);
    // Serialize message field [sdnn]
    bufferOffset = _serializer.uint8(obj.sdnn, buffer, bufferOffset);
    // Serialize message field [rmssd]
    bufferOffset = _serializer.uint8(obj.rmssd, buffer, bufferOffset);
    // Serialize message field [nn50]
    bufferOffset = _serializer.uint8(obj.nn50, buffer, bufferOffset);
    // Serialize message field [pnn50]
    bufferOffset = _serializer.uint8(obj.pnn50, buffer, bufferOffset);
    // Check that the constant length array field [rra] has the right length
    if (obj.rra.length !== 6) {
      throw new Error('Unable to serialize array field rra - length must be 6')
    }
    // Serialize message field [rra]
    bufferOffset = _arraySerializer.uint8(obj.rra, buffer, bufferOffset, 6);
    // Serialize message field [rsv2]
    bufferOffset = _serializer.uint8(obj.rsv2, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type personData
    let len;
    let data = new personData(null);
    // Deserialize message field [header]
    data.header = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [acdata]
    data.acdata = _arrayDeserializer.int8(buffer, bufferOffset, 64)
    // Deserialize message field [heartrate]
    data.heartrate = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [spo2]
    data.spo2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [bk]
    data.bk = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rsv1]
    data.rsv1 = _arrayDeserializer.uint8(buffer, bufferOffset, 8)
    // Deserialize message field [sdnn]
    data.sdnn = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rmssd]
    data.rmssd = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [nn50]
    data.nn50 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pnn50]
    data.pnn50 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rra]
    data.rra = _arrayDeserializer.uint8(buffer, bufferOffset, 6)
    // Deserialize message field [rsv2]
    data.rsv2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'qt_cmd_vel/personData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0b9dfbf73ccdfbf3d5b1a87f99434849';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 header        # 0xFF数据包头
    int8[64] acdata     # 心律波形数据 (有符号)
    uint8 heartrate     # 心率
    uint8 spo2          # 血氧
    uint8 bk            # 微循环
    uint8[8] rsv1       # 保留数据 1
    uint8 sdnn          # 心率变异性
    uint8 rmssd
    uint8 nn50
    uint8 pnn50
    uint8[6] rra        # RR间期
    uint8 rsv2          # 保留数据 2
    uint8 state         # 模块状态
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new personData(null);
    if (msg.header !== undefined) {
      resolved.header = msg.header;
    }
    else {
      resolved.header = 0
    }

    if (msg.acdata !== undefined) {
      resolved.acdata = msg.acdata;
    }
    else {
      resolved.acdata = new Array(64).fill(0)
    }

    if (msg.heartrate !== undefined) {
      resolved.heartrate = msg.heartrate;
    }
    else {
      resolved.heartrate = 0
    }

    if (msg.spo2 !== undefined) {
      resolved.spo2 = msg.spo2;
    }
    else {
      resolved.spo2 = 0
    }

    if (msg.bk !== undefined) {
      resolved.bk = msg.bk;
    }
    else {
      resolved.bk = 0
    }

    if (msg.rsv1 !== undefined) {
      resolved.rsv1 = msg.rsv1;
    }
    else {
      resolved.rsv1 = new Array(8).fill(0)
    }

    if (msg.sdnn !== undefined) {
      resolved.sdnn = msg.sdnn;
    }
    else {
      resolved.sdnn = 0
    }

    if (msg.rmssd !== undefined) {
      resolved.rmssd = msg.rmssd;
    }
    else {
      resolved.rmssd = 0
    }

    if (msg.nn50 !== undefined) {
      resolved.nn50 = msg.nn50;
    }
    else {
      resolved.nn50 = 0
    }

    if (msg.pnn50 !== undefined) {
      resolved.pnn50 = msg.pnn50;
    }
    else {
      resolved.pnn50 = 0
    }

    if (msg.rra !== undefined) {
      resolved.rra = msg.rra;
    }
    else {
      resolved.rra = new Array(6).fill(0)
    }

    if (msg.rsv2 !== undefined) {
      resolved.rsv2 = msg.rsv2;
    }
    else {
      resolved.rsv2 = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    return resolved;
    }
};

module.exports = personData;
