
"use strict";

let MotorCommand = require('./MotorCommand.js');
let Altimeter = require('./Altimeter.js');
let Compass = require('./Compass.js');
let RuddersCommand = require('./RuddersCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let ControllerState = require('./ControllerState.js');
let RawImu = require('./RawImu.js');
let HeadingCommand = require('./HeadingCommand.js');
let MotorStatus = require('./MotorStatus.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let RC = require('./RC.js');
let Supply = require('./Supply.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let HeightCommand = require('./HeightCommand.js');
let MotorPWM = require('./MotorPWM.js');
let RawRC = require('./RawRC.js');
let ServoCommand = require('./ServoCommand.js');
let YawrateCommand = require('./YawrateCommand.js');
let ThrustCommand = require('./ThrustCommand.js');
let RawMagnetic = require('./RawMagnetic.js');

module.exports = {
  MotorCommand: MotorCommand,
  Altimeter: Altimeter,
  Compass: Compass,
  RuddersCommand: RuddersCommand,
  VelocityXYCommand: VelocityXYCommand,
  ControllerState: ControllerState,
  RawImu: RawImu,
  HeadingCommand: HeadingCommand,
  MotorStatus: MotorStatus,
  PositionXYCommand: PositionXYCommand,
  VelocityZCommand: VelocityZCommand,
  RC: RC,
  Supply: Supply,
  AttitudeCommand: AttitudeCommand,
  HeightCommand: HeightCommand,
  MotorPWM: MotorPWM,
  RawRC: RawRC,
  ServoCommand: ServoCommand,
  YawrateCommand: YawrateCommand,
  ThrustCommand: ThrustCommand,
  RawMagnetic: RawMagnetic,
};
