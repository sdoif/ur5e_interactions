
"use strict";

let GetRobotMode = require('./GetRobotMode.js')
let AddToLog = require('./AddToLog.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let RawRequest = require('./RawRequest.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetProgramState = require('./GetProgramState.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let Popup = require('./Popup.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let Load = require('./Load.js')

module.exports = {
  GetRobotMode: GetRobotMode,
  AddToLog: AddToLog,
  IsInRemoteControl: IsInRemoteControl,
  IsProgramSaved: IsProgramSaved,
  RawRequest: RawRequest,
  GetSafetyMode: GetSafetyMode,
  GetProgramState: GetProgramState,
  GetLoadedProgram: GetLoadedProgram,
  Popup: Popup,
  IsProgramRunning: IsProgramRunning,
  Load: Load,
};
