
'use strict'
console.log('Loading AS200')

var tools = {}
var cogUtils = {}
var cogMath = {}

  tools = process.binding('InSight.Tools')
  cogUtils = require('CogUtils.js')
cogMath = require('CogMath.js')

const version = Object.freeze(new cogUtils.Version(3, 3, 0,""))
console.log('AS200 Version: ' + version.toString())

// Logger
const myLog = cogUtils.log
const asLogger = new Logger()

const MAX_LOGGING_ENTRIES = 60

// System
const MAX_LOOPS = 3 // max steps for auto-calibration if the feature is out of fov

const MIN_POSES = 11  // minimum number of poses for calibration 
const MIN_POSES_ROTATED = 2 // minimum number of rotated poses for calibration
const MIN_ANGLE_RANGE = 0.09
const MIN_DIST_PIXEL = 50 //minimum distance in pixels between the pre-calibration
const START_MOVE_DISTANCE_MM = 1 // start value for the movement of the robot during the pre-calibration. Is only needed for the old spread sheet without "First Step Size"

// Camera ID      0000 0000 xxxx xxxx 0 - 255
// Part 1         0000 0000 0000 xxxx 0 - 15
// Part 2         0000 0000 xxxx 0000 0 - 15
// Gripper 1      0000 xxxx 0000 0000

// Moving Camera  0000 1000 0000 0000

// Constants for the different ID-decoding
const MASK_CAM_ID = 0xff
const MASK_GRIPPER_ID_1 = 0xf00

const MASK_ID_1 = 0x0f
const MASK_ID_2 = 0xf0
const SHIFT_ID_1 = 0
const SHIFT_ID_2 = 4

const SHIFT_GRIPPER_ID_1 = 8
const SHIFT_MOVING_CAMERA = 11

// Mask for encoding the moving camera bit 
const MASK_MOVING_CAMERA = 0x800

// Maximum numbers of calibrations
const MAX_CALIBRATIONS = 2

// Definitions for the system
const MAX_CAMERAS = 2
const MAX_FEATURES_PER_CAMERA = 4
const MAX_GRIPPERS = 4
const MAX_BACKUP_PARTS = 8

const CAMERACONFIG_FILENAME = 'CameraConfig.cxd'
const SENSORCONFIG_FILENAME = 'SensorConfig.cxd'
// Communication
const INTERNAL_PORT = 7891

// For the recipe-tables
const RECIPE_MAX_STEPS = 12
const RECIPE_MAX_FEATURES = 12
const RECIPE_MAX_PARTS = 12
const RECIPE_STEPTABLE = 'Recipe.StepTable'
const RECIPE_FEATURETABLE = 'Recipe.FeatureTable'
const RECIPE_PARTTABLE = 'Recipe.PartTable'

/**
 * this function is only used for internal tracking 
 * of the execution time of functions * 
 * */
var timeTracker = (function () {
  var startTime = process.hrtime()
  return {
    restart: function () {
      startTime = process.hrtime()
    },
    getElapsedTime: function () {
      let dif = process.hrtime(startTime)
      return ((dif[0] + dif[1] / 1000000000).toFixed(6))
    }
  }
})()

/**
 * this function creates an internal logging buffer.
 * This is also only for internal use.
 * It is used to trace all the functions which are called
 * from receiving a command to sending the result to the PLC*/
var tracer = (function () {
  var messages = []
  return {
    clear: function () {
      messages = []
    },
    addMessage: function (message) {
      messages.push(message)
      if (messages.length > 150) {
        messages.splice(0, messages.length - 150)
      }
    },
    print: function () {
      messages.forEach(function (message) {
        myLog(message)
      })
    }
  }
})()

// Holds all registerd inspections
var g_Inspections = {}

// Variable for the all settings for the custom calibration
var g_CustomCalibrationSettings = null
// Variable for the all settings for the HE calibration
var g_HECalibrationSettings = null

// These variables are needed for plotting the cross hair
// Here are only the default values for start up
var g_HRes = 1280
var g_VRes = 960
var g_Border = [[0, 0, 0, 1280], [0, 1280, 960, 1280], [960, 1280, 960, 0], [960, 0, 0, 0]]

// Holds the information for the cross hairs
// Position and enabled/disabled
var g_Graphics = { 'ShowCalibrationPoints_1': 0, 'ShowCalibrationPoints_2': 0, 'ShowCrossHair': null }

// Holds all informations from the camera configuration file
var g_Settings = {
  'UseShuttledCameras': false,
  'Sensor': null,
  'Cams': null,
  'JobServer': null,
  'Communication': null,
  'AutoCalibration': null,
  'LogImageFtp': null,
  'LogDataFtp': null,
  'CustomCalibration': null,
  'IsRobotMounted': null,
  'RunUncalibrated':null
}

// Contains the pixel values of the enabled features after "toolsDone"
var g_CurrentFeatures = null

// Holds all the information for each feature from the feature table
// cameraIsMoving":false,
// "partIsMoving": true, 
// "shuttlePose": 1,
// "partID": 1
var g_FeaturesInfos = null

// Transformed position of the features from the feature table
// "valid":1,
// "x": 115.73103390010152, 
// "y": 96.4864257983316, 
// "thetaInDegrees": -82.99123098829205
var g_RuntimeFeatures = null

// Trained positions of each feature from the feature table
// Each feature can have four positions. Each gripper needs his own position
// {"1":[{"valid":1,"x":129.52149080537544,"y":90.63357546839163,"thetaInDegrees":-91.60653879542978},
//       {"valid": 0, "x": 0, "y": 0, "thetaInDegrees": 0 },
//       {"valid": 0, "x": 0, "y": 0, "thetaInDegrees": 0 },
//       {"valid": 0, "x": 0, "y": 0, "thetaInDegrees": 0 }],
// "2": [{ "valid": 1, "x": 122.62381643195218, "y": -86.70209363412792, "thetaInDegrees": -91.58384337332423 },
// ....}]}
var g_TrainedFeatures = null

// Same as TrainedFeatures but for the robot coordinates
var g_TrainedRobotPoses = null

// Contains the stored trained feature poses
var g_StoredTrainedFeatures = null
// Contains the stored trained robot poses
var g_StoredTrainedRobotPoses = null

// used for the grip correction functionality 
var g_GripCorrections = null
// used for the frame correction functionality
var g_FrameCorrections = null

// only used during the calibration cycle
var g_AutoCalibRuntime = {}

// Holds all the calibrations, including the feature and robot positions
var g_Calibrations = null

// G_LookupTables includes all three tables from the spread sheet
// (Part Table, Feature Table and Step table)
// and it holds also the lookup tables for the different ID types (camera, part or step).
// The new generated tables holds all informations for a command 
// (exposure set, enabled features, used cameras, ...)
var g_LooukupTables = null

// Points to the new generated "StepsLookup table"
// Is a subset of g_LookupTables
var g_Steps = null

// Holds the runtime and trained poses for the parts.
// The grip correction is alos stored in this variable
var g_Parts = null

//* ***************************************************************************/
var RobotPose = (function () {
  function RobotPose(x, y, z, thetaZ, thetaY, thetaX, valid) {
    this.valid = valid
    this.x = x
    this.y = y
    this.z = z
    this.thetaZ = thetaZ
    this.thetaY = thetaY
    this.thetaX = thetaX
  }
  return RobotPose
}())
RobotPose.prototype.getAsString = function () {
  let res = InSightFunctions.fnStringf('%.5f,%.5f,%.5f,%.5f,%.5f,%.5f', this.x, this.y, this.z, this.thetaZ, this.thetaY, this.thetaX)
  return res
}

/**
 * Holds all information about the feature
 * */
var FeatureInfo = (function () {
  function FeatureInfo(cameraIsMoving, partIsMoving, shuttlePose, partID) {
    this.cameraIsMoving = cameraIsMoving
    this.partIsMoving = partIsMoving
    this.shuttlePose = shuttlePose
    this.partID = partID
  }
  return FeatureInfo
}())

/**
 * Container for Custom-Calibration settings
 * */
var CustomCalibrationSettings = (function () {
  function CustomCalibrationSettings(swapHandedness, featureOffsetX, featureOffsetY, calibrationID) {
    this.swapHandedness = swapHandedness
    this.featureOffsetX = featureOffsetX
    this.featureOffsetY = featureOffsetY
    this.calibrationID = calibrationID
  }
  return CustomCalibrationSettings
}())

/**
 * Container for HE-Calibration settings
 * */
var HECalibrationSettings = (function () {
  function HECalibrationSettings() {
    this.calibrationID = -1
  }
  return HECalibrationSettings
}())

function Result() {
  this.state = 0 // contains the state of the operation/result => OK/nOK
  this.isValid = 0 // the result structure was updated, the data are ready to use
  this.isNeeded = 0
  this.data = []
}


/**
 * Contains the feature coordinates
 * */
var Feature = (function () {
  function Feature(x, y, thetaInDegrees, valid) {
    this.valid = valid
    this.x = x
    this.y = y
    this.thetaInDegrees = thetaInDegrees
  }
  return Feature
}())

Feature.prototype.reset = function () {
  this.x = 0
  this.y = 0
  this.thetaInDegrees = 0
  this.valid = 0
}
//* ***************************************************************************/
// -> From Guru
var CogCalibFixComputationModeConstants;
(function (CogCalibFixComputationModeConstants) {
  CogCalibFixComputationModeConstants[CogCalibFixComputationModeConstants['Linear'] = 1] = 'Linear'
  CogCalibFixComputationModeConstants[CogCalibFixComputationModeConstants['PerspectiveAndRadialWarp'] = 2] = 'PerspectiveAndRadialWarp'
  CogCalibFixComputationModeConstants[CogCalibFixComputationModeConstants['LinescanWarp'] = 3] = 'LinescanWarp'
  CogCalibFixComputationModeConstants[CogCalibFixComputationModeConstants['Linescan2DWarp'] = 4] = 'Linescan2DWarp'
  CogCalibFixComputationModeConstants[CogCalibFixComputationModeConstants['SineTanLawProjectionWarp'] = 5] = 'SineTanLawProjectionWarp'
  CogCalibFixComputationModeConstants[CogCalibFixComputationModeConstants['ThreeParamRadialWarp'] = 6] = 'ThreeParamRadialWarp'
  CogCalibFixComputationModeConstants[CogCalibFixComputationModeConstants['NoDistortionWarp'] = 7] = 'NoDistortionWarp'
})(CogCalibFixComputationModeConstants || (CogCalibFixComputationModeConstants = {}))

// ReSharper disable once InconsistentNaming
var CogNPointToNPointDOFConstants;
(function (CogNPointToNPointDOFConstants) {
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['None'] = 0] = 'None'
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['TranslationX'] = 1] = 'TranslationX'
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['TranslationY'] = 2] = 'TranslationY'
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['Translation'] = 3] = 'Translation'
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['RotationAndTranslation'] = 4] = 'RotationAndTranslation'
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['ScalingRotationAndTranslation'] = 5] = 'ScalingRotationAndTranslation'
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['ScalingAspectRotationAndTranslation'] = 6] = 'ScalingAspectRotationAndTranslation'
  CogNPointToNPointDOFConstants[CogNPointToNPointDOFConstants['ScalingAspectRotationSkewAndTranslation'] = 7] = 'ScalingAspectRotationSkewAndTranslation'
})(CogNPointToNPointDOFConstants || (CogNPointToNPointDOFConstants = {}))

var CogNPointToNPoint = (function () {
  function CogNPointToNPoint() {
    this.groupAPoints = []
    this.groupBPoints = []
    this.computationMode = CogCalibFixComputationModeConstants.Linear
    this.dofsToCompute = CogNPointToNPointDOFConstants.RotationAndTranslation
    this.S_ = {
      n: 0,
      sum_u: 0,
      sum_v: 0,
      sum_u2: 0,
      sum_v2: 0,
      sum_uv: 0,
      sum_x: 0,
      sum_y: 0,
      sum_x2: 0,
      sum_y2: 0,
      sum_xy: 0,
      sum_ux: 0,
      sum_uy: 0,
      sum_vx: 0,
      sum_vy: 0
    }
  }
  Object.defineProperty(CogNPointToNPoint.prototype, 'NumPoints', {
    // Constructor
    // Public properties
    // ReSharper disable once InconsistentNaming
    get: function () {
      return this.groupAPoints.length
    },
    enumerable: true,
    configurable: true
  })
  Object.defineProperty(CogNPointToNPoint.prototype, 'ComputationMode', {
    // ReSharper disable once InconsistentNaming
    get: function () {
      return this.computationMode
    },
    // ReSharper disable once InconsistentNaming
    set: function (val) {
      if (val !== CogCalibFixComputationModeConstants.Linear) { throw new Error('Only Linear mode is supported') }
      this.computationMode = val
    },
    enumerable: true,
    configurable: true
  })
  Object.defineProperty(CogNPointToNPoint.prototype, 'DOFsToCompute', {
    // ReSharper disable once InconsistentNaming
    get: function () {
      return this.dofsToCompute
    },
    // ReSharper disable once InconsistentNaming
    set: function (val) {
      if (val !== CogNPointToNPointDOFConstants.RotationAndTranslation) { throw new Error('Only RotationAndTranslation mode is supported') }
      this.dofsToCompute = val
    },
    enumerable: true,
    configurable: true
  })

  // Public Methods
  // ReSharper disable once InconsistentNaming
  CogNPointToNPoint.prototype.AddPointPair = function (groupAPoint, groupBPoint) {
    if (groupAPoint.valid && groupBPoint.valid) {
      this.groupAPoints.push(groupAPoint)
      this.groupBPoints.push(groupBPoint)
      var u = groupAPoint.x
      var v = groupAPoint.y
      var x = groupBPoint.x
      var y = groupBPoint.y
      this.S_.sum_u += u
      this.S_.sum_v += v
      this.S_.sum_u2 += u * u
      this.S_.sum_v2 += v * v
      this.S_.sum_uv += u * v
      this.S_.sum_x += x
      this.S_.sum_y += y
      this.S_.sum_x2 += x * x
      this.S_.sum_y2 += y * y
      this.S_.sum_xy += x * y
      this.S_.sum_ux += u * x
      this.S_.sum_vx += v * x
      this.S_.sum_uy += u * y
      this.S_.sum_vy += v * y
      this.S_.n++
    }
  }
  CogNPointToNPoint.prototype.ComputeGroupAFromGroupBTransform = function () {
    var eps = 1e-15
    var n = this.S_.n
    if (n < 2) { throw new Error('Unstable Error!') }
    var c1 = n * (this.S_.sum_ux + this.S_.sum_vy) - this.S_.sum_u * this.S_.sum_x - this.S_.sum_v * this.S_.sum_y
    var c2 = n * (this.S_.sum_vx - this.S_.sum_uy) + this.S_.sum_u * this.S_.sum_y - this.S_.sum_v * this.S_.sum_x
    var angle = Math.atan2(c2, c1)
    if (Math.abs(c1) < eps && Math.abs(c2) < eps) { throw new Error('Unstable Error!') }
    var s = Math.sin(angle)
    var c = Math.cos(angle)
    var x = (this.S_.sum_u - c * this.S_.sum_x + s * this.S_.sum_y) / n
    var y = (this.S_.sum_v - s * this.S_.sum_x - c * this.S_.sum_y) / n
    var groupAFromGroupBTransform = new cogMath.cc2XformLinear()
    groupAFromGroupBTransform.setXform([[c, -s, x], [s, c, y]])
    var rmsError = 0
    for (var i = 0; i < this.groupBPoints.length; i++) {
      var mappedPoint = groupAFromGroupBTransform.mapPoint([this.groupBPoints[i].x, this.groupBPoints[i].y])
      var xError = this.groupAPoints[i].x - mappedPoint[0]
      var yError = this.groupAPoints[i].y - mappedPoint[1]
      rmsError += xError * xError + yError * yError
    }

    rmsError = Math.sqrt(rmsError / this.groupBPoints.length)
    return { groupAFromGroupBTransform: groupAFromGroupBTransform, rmsError: rmsError }
  }
  return CogNPointToNPoint
}())
function ComputeDesiredStagePosition(trainFeatures, runFeatures, heCalibResultJSON, unCorrectedStagePoseX, unCorrectedStagePoseY, unCorrectedStagePoseThetaDeg) {
  if (runFeatures.length !== trainFeatures.length) { throw new Error('The number of train features is not equal to the number of run features') }
  var heCalibResult = new cogMath.ccMultiViewPlanarMotionCalibResult()
  // heCalibResult.initFromObject(JSON.parse(heCalibResultJSON));
  heCalibResult.initFromObject(heCalibResultJSON)
  // Convert stage position to corrected Home2D
  var home2DFromStage2DUnCorrected = new cogMath.cc2Rigid()
  home2DFromStage2DUnCorrected.setXform(unCorrectedStagePoseThetaDeg, [unCorrectedStagePoseX, unCorrectedStagePoseY])
  var home2DFromStage2D = heCalibResult
    .convertUncorrectedHome2DFromStage2DToHome2DFromStage2D(home2DFromStage2DUnCorrected)
  var stage2DFromHome2D = home2DFromStage2D.inverse()
  var xform = {}
  if (trainFeatures.length >= 2) {
    // Map run time features to position when stage is at home
    var remappedRunFeatures = []
    for (var _i = 0, runFeatures_1 = runFeatures; _i < runFeatures_1.length; _i++) {
      var feature = runFeatures_1[_i]
      var mappedFeature = stage2DFromHome2D.mapPoint([feature.x, feature.y])
      remappedRunFeatures.push(new Feature(mappedFeature[0], mappedFeature[1], 0, feature.valid))
    }
    // Compute trainFromRun transform. This is the desire position of the stage
    var nPointToNPoint = new CogNPointToNPoint()
    for (var i = 0; i < trainFeatures.length; i++) {
      nPointToNPoint.AddPointPair(trainFeatures[i], remappedRunFeatures[i])
    }
    // var rmsError = void 0
    xform = nPointToNPoint.ComputeGroupAFromGroupBTransform()
  } else if (trainFeatures.length === 1) {
    var home2DRunFromPart2D = new cogMath.cc2Rigid()
    home2DRunFromPart2D.setXform(runFeatures[0].thetaInDegrees, [runFeatures[0].x, runFeatures[0].y])
    var stage2DRunFromPart2D = stage2DFromHome2D.compose(home2DRunFromPart2D)
    var trainFromPart = new cogMath.cc2XformLinear()
    trainFromPart.setXformScaleRotation(trainFeatures[0].thetaInDegrees, trainFeatures[0].thetaInDegrees, 1.0, 1.0, [trainFeatures[0].x, trainFeatures[0].y])
    var trainFromRun = trainFromPart.compose(stage2DRunFromPart2D.inverse())
    var trainFromRunLinear = new cogMath.cc2XformLinear()
    trainFromRunLinear.setXformScaleRotation(trainFromRun.rotationInDegrees(), trainFromRun.rotationInDegrees(), 1.0, 1.0, trainFromRun.trans())
    xform = { groupAFromGroupBTransform: trainFromRunLinear, rmsError: 0.0 }
  } else { throw new Error('At least two valid point pairs should be added') }

  // Convert to uncorrected Home2D
  var desiredHome2DFromStage2DUnCorrected = new cogMath.cc2Rigid()
  desiredHome2DFromStage2DUnCorrected.setXform(xform['groupAFromGroupBTransform'].rotationInDegrees(), xform['groupAFromGroupBTransform'].trans())
  var desiredHome2DFromStage2DCorrected = heCalibResult
    .convertUncorrectedHome2DFromStage2DToHome2DFromStage2D(desiredHome2DFromStage2DUnCorrected)
  // Generate results object
  var result = {}
  result['desiredHome2DFromStage2DCorrected'] = desiredHome2DFromStage2DCorrected
  result['rmsError'] = xform['rmsError']
  return result
}
// <- From Guru
//* ***************************************************************************/

//* ***************************************************************************/
//* ***************************************************************************/
// Definitions
//* ***************************************************************************/


/**
 * Resolutions
 * */
var Resolutions;
(function (Resolutions) {
  Resolutions[Resolutions['Full'] = 1] = 'Full'
  Resolutions[Resolutions['Half'] = 2] = 'Half'
  Resolutions[Resolutions['Quarter'] = 3] = 'Quarter'
})(Resolutions || (Resolutions = {}))

/**
 * States
 * */
var States;
(function (States) {
  States[States['WAITING_FOR_NEW_COMMAND'] = 1] = 'WAITING_FOR_NEW_COMMAND'
  States[States['WAITING_FOR_IMAGE_ACQUIRED'] = 2] = 'WAITING_FOR_IMAGE_ACQUIRED'
  States[States['WAITING_FOR_TOOLS_DONE'] = 3] = 'WAITING_FOR_TOOLS_DONE'
  States[States['WAITING_FOR_SLAVE_RESULT'] = 4] = 'WAITING_FOR_SLAVE_RESULT'
  States[States['REDO_FROM_EXECUTE'] = 5] = 'REDO_FROM_EXECUTE'
})(States || (States = {}))

/**
 * Error codes
 * */
var ECodes;
(function (ECodes) {
  ECodes[ECodes['E_NO_ERROR'] = 99999] = 'E_NO_ERROR'
  ECodes[ECodes['E_UNSPECIFIED'] = 0] = 'E_UNSPECIFIED'

  ECodes[ECodes['E_TIMEOUT'] = -1000] = 'E_TIMEOUT'
  ECodes[ECodes['E_UNKNOWN_COMMAND'] = -1001] = 'E_UNKNOWN_COMMAND'
  ECodes[ECodes['E_INDEX_OUT_OF_RANGE'] = -1002] = 'E_INDEX_OUT_OF_RANGE'
  ECodes[ECodes['E_TOO_FEW_ARGUMENTS'] = -1003] = 'E_TOO_FEW_ARGUMENTS'
  ECodes[ECodes['E_INVALID_ARGUMENT'] = -1005] = 'E_INVALID_ARGUMENT'
  ECodes[ECodes['E_COMMAND_NOT_ALLOWED'] = -1006] = 'E_COMMAND_NOT_ALLOWED'
  ECodes[ECodes['E_COMBINATION_NOT_ALLOWED'] = -1007] = 'E_COMBINATION_NOT_ALLOWED'
  ECodes[ECodes['E_BUSSY'] = -1008] = 'E_BUSSY'
  ECodes[ECodes['E_NOT_FULLY_IMPLEMENTED'] = -1009] = 'E_NOT_FULLY_IMPLEMENTED'
  ECodes[ECodes['E_NOT_SUPPORTED'] = -1010] = 'E_NOT_SUPPORTED'
  ECodes[ECodes['E_RESULSTRING_TO_LONG'] = -1011] = 'E_RESULSTRING_TO_LONG'
  ECodes[ECodes['E_INVALID_CAMERA_ID'] = -1012] = 'E_INVALID_CAMERA_ID'
  ECodes[ECodes['E_INVALID_CAMERA_FEATURE_ID'] = -1013] = 'E_INVALID_CAMERA_FEATURE_ID'
  ECodes[ECodes['E_INVALID_RESULT_MODE'] = -1014] = 'E_INVALID_RESULT_MODE'
  ECodes[ECodes['E_INVALID_LCHECK_RESULT_MODE'] = -1015] = 'E_INVALID_LCHECK_RESULT_MODE'
  ECodes[ECodes['E_INVALID_COORDINATE_SYSTEM'] = -1016] = 'E_INVALID_COORDINATE_SYSTEM'
  ECodes[ECodes['E_INVALID_RESOLUTION'] = -1017] = 'E_INVALID_RESOLUTION'
  ECodes[ECodes['E_INVALID_ARG_TYPE_DEFINITION'] = -1018] = 'E_INVALID_ARG_TYPE_DEFINITION'

  ECodes[ECodes['E_DIFFERENT_JOB_NAMES'] = -1101] = 'E_DIFFERENT_JOB_NAMES'
  ECodes[ECodes['E_DIFFERENT_VERSIONS'] = -1102] = 'E_DIFFERENT_VERSIONS'

  ECodes[ECodes['E_NOT_CALIBRATED'] = -2001] = 'E_NOT_CALIBRATED'
  ECodes[ECodes['E_CALIBRATION_FAILED'] = -2002] = 'E_CALIBRATION_FAILED'
  ECodes[ECodes['E_INVALID_CALIBRATION_DATA'] = -2003] = 'E_INVALID_CALIBRATION_DATA'
  ECodes[ECodes['E_NOT_GIVEN_CALIBRATION_POSE'] = -2004] = 'E_NOT_GIVEN_CALIBRATION_POSE'
  ECodes[ECodes['E_NO_START_COMMAND'] = -2005] = 'E_NO_START_COMMAND'

  ECodes[ECodes['E_FEATURE_NOT_TRAINED'] = -3001] = 'E_FEATURE_NOT_TRAINED'
  ECodes[ECodes['E_FEATURE_NOT_FOUND'] = -3002] = 'E_FEATURE_NOT_FOUND'
  ECodes[ECodes['E_FEATURE_NOT_MAPPED'] = -3003] = 'E_FEATURE_NOT_MAPPED'
  ECodes[ECodes['E_TARGET_POSE_NOT_TRAINED'] = -3004] = 'E_TARGET_POSE_NOT_TRAINED'
  ECodes[ECodes['E_ROBOT_POSE_NOT_TRAINED'] = -3005] = 'E_ROBOT_POSE_NOT_TRAINED'
  ECodes[ECodes['E_NO_GC_RESULT'] = -3006] = 'E_NO_GC_RESULT'

  ECodes[ECodes['E_INVALID_PART_ID'] = -4001] = 'E_INVALID_PART_ID'
  ECodes[ECodes['E_PART_NOT_ALL_FEATURES_LOCATED'] = -4002] = 'E_PART_NOT_ALL_FEATURES_LOCATED'
  ECodes[ECodes['E_PART_NO_VALID_GRIP_CORRECTION'] = -4003] = 'E_PART_NO_VALID_GRIP_CORRECTION'
  ECodes[ECodes['E_PART_NO_VALID_FRAME_CORRECTION'] = -4004] = 'E_PART_NO_VALID_FRAME_CORRECTION'

  ECodes[ECodes['E_INTERNAL_ERROR'] = -9999] = 'E_INTERNAL_ERROR'
})(ECodes || (ECodes = {}))

/**
 * Coordinate Systems
 * */
var CoordinateSystem;
(function (CoordinateSystem) {
  CoordinateSystem[CoordinateSystem['HOME2D'] = 1] = 'HOME2D'
  CoordinateSystem[CoordinateSystem['CAM2D'] = 2] = 'CAM2D'
  CoordinateSystem[CoordinateSystem['RAW2D'] = 3] = 'RAW2D'
})(CoordinateSystem || (CoordinateSystem = {}))

/**
 * Result modes
 * */
var ResultMode;
(function (ResultMode) {
  ResultMode[ResultMode['ABS'] = 1] = 'ABS'
  ResultMode[ResultMode['OFF'] = 2] = 'OFF'
  ResultMode[ResultMode['FRAME'] = 3] = 'FRAME'
  ResultMode[ResultMode['PICKED'] = 4] = 'PICKED'
  ResultMode[ResultMode['GC'] = 5] = 'GC'
  ResultMode[ResultMode['GCP'] = 6] = 'GCP'
})(ResultMode || (ResultMode = {}))

/**
 * LCheck ResultType
 * */
var LCheckResultMode;
(function (LCheckResultMode) {
  LCheckResultMode[LCheckResultMode['ABS'] = 1] = 'ABS'
  LCheckResultMode[LCheckResultMode['DIFF'] = 2] = 'DIFF'
  LCheckResultMode[LCheckResultMode['REL'] = 3] = 'REL'
})(LCheckResultMode || (LCheckResultMode = {}))

/**
 * Type definitions for argument check
 * */
var ARG_TYPES;
(function (ARG_TYPES) {
  ARG_TYPES[ARG_TYPES['INT'] = 1] = 'INT'
  ARG_TYPES[ARG_TYPES['FLOAT'] = 2] = 'FLOAT'
  ARG_TYPES[ARG_TYPES['STRING'] = 3] = 'STRING'
  ARG_TYPES[ARG_TYPES['DEF_COORDINATE_SYS'] = 4] = 'DEF_COORDINATE_SYS'
  ARG_TYPES[ARG_TYPES['DEF_RESULT_MODE'] = 5] = 'DEF_RESULT_MODE'
  ARG_TYPES[ARG_TYPES['DEF_LCHECK_RESULT_MODE'] = 6] = 'DEF_LCHECK_RESULT_MODE'
})(ARG_TYPES || (ARG_TYPES = {}))
//* ***************************************************************************/
// End of Definitions
//* ***************************************************************************/
//* ***************************************************************************/

//* ***************************************************************************/
// AS200
//* ***************************************************************************/

/**
 * Initialize the AS200 script
 * this is called from the In-Sight through B6
 * */
function AS200() {  
  timeTracker.restart()
  myLog('AS200 Script: -> Init!')

  this.counter = 0

  this.firstRunDone = 0
  this.firstTimeOnlineDone = 0
  this.autoExposureState = 0

  this.currentFeatureMask = 1
  this.triggerMode = 32
  // Contains Informations about the camera (IP, Name, Master, JobName, SLMP, Index,...)
  this.myDetails = new MyDetails()

  // Contains the Informations from the SensorConfig file (only Master)
  this.mySensor = null // Sensor information from the SensorConfig file
  this.myRecipes = null // all recipes stored in the SensorConfig file

  this.mySensorConfiguration = null// new SensorConfiguration();

  g_LooukupTables = null

  this.myCalibrations = 0
  this.commands = null
  this.slmpMap = null
  this.currentCommand = null
  this.lastCommand = null
  this.currentState = States.WAITING_FOR_NEW_COMMAND

  this.myParts = null

  this.mySharedObjects = SharedObjects.getInstance()
  this.mySharedObjects.addSharedObject('AS200', this)

  for (let i in masterCommands) {
    if (typeof masterCommands[i] === 'function') {
      inheritPseudoClass(CommandBase, masterCommands[i])
    }
  }

  for (let i in slaveCommands) {
    if (typeof slaveCommands[i] === 'function') {
      inheritPseudoClass(SlaveBase, slaveCommands[i])
    }
  }
  myLog('AS200 init done ' + timeTracker.getElapsedTime())
};

/**
 * This is the run function of the AS200 script
 * This is called from the In-Sight through B6 if the online state is changing
 * This functions loads the slave job if the sensor goes online (only at the first time)
 * @param {any} image  Was used in the first version (script and job) but not longer needed
 * @param {any} online The current online/offline state 
 */
AS200.prototype.run = function (image, online) {
  tracer.addMessage('-> AS200 Script: Run ' + timeTracker.getElapsedTime())
  if (this.firstRunDone === 0) {
    this.firstRun()
  } else if ((online === 1) && (this.firstRunDone === 1) && (this.firstTimeOnlineDone === 0)) {
    this.firstTimeOnline()
  } else if (online === 0) {
    this.currentState = States.WAITING_FOR_NEW_COMMAND
    this.currentCommand = null
  }

  tracer.addMessage('<- AS200 Script: Run ' + timeTracker.getElapsedTime())
  return this.currentState
}

/**
 * Is called from the In-Sight
 * sames as a script inside the spread sheet
 * */
AS200.prototype.save = function () {
  myLog('-> AS200 Save data ')
  if (g_TrainedFeatures == null) {
    myLog('===>')
    myLog('g_TrainedFeatures == null')
    myLog('<===')
  }

  if (g_TrainedRobotPoses == null) {
    myLog('===>')
    myLog('g_TrainedRobotPoses == null')
    myLog('<===')
  }


  myLog(g_TrainedFeatures)
  myLog(g_TrainedRobotPoses)

  g_StoredTrainedFeatures = g_TrainedFeatures
  g_StoredTrainedRobotPoses = g_TrainedRobotPoses

  myLog('<- AS200 Save data')
  return {
    'g_StoredTrainedFeatures': JSON.stringify(g_TrainedFeatures),
    'g_StoredTrainedRobotPoses': JSON.stringify(g_TrainedRobotPoses)
  }
}
/**
 * Is called from the In-Sight
 * sames as a script inside the spread sheet
 * @param {any} saved
 */
AS200.prototype.load = function (saved) {
  myLog('-> AS200 Load stored data')
  if (saved == null) {
    saved = {}
    saved["g_StoredTrainedFeatures"] = JSON.stringify(createDefaultFeaturesData())
    saved["g_StoredTrainedRobotPoses"] = JSON.stringify(createDefaultRobotPosesData())
  }
  g_StoredTrainedFeatures = JSON.parse(saved.g_StoredTrainedFeatures)
  g_StoredTrainedRobotPoses = JSON.parse(saved.g_StoredTrainedRobotPoses)
  myLog('<- AS200 Load stored data')
}
/**
 * Is called from the In-Sight
 * sames as a script insight the spread sheet
 * This functions draws the calibration points 
 * and the cross hair
 * @param {any} gr graphic object from In-Sight
 */
AS200.prototype.drawGraphics = function (gr) {
  let colors = [0xff0000, 0xff00ff]
  tracer.addMessage('-> Draw ' + timeTracker.getElapsedTime())

  if (g_Graphics.ShowCalibrationPoints_1 > 0) {
    
    let targetX = this.myCalibrations.calibrations[1].calibrationData.targetX
    let targetY = this.myCalibrations.calibrations[1].calibrationData.targetY

    for (var i = 0; i < targetX.length; i++) {
      gr.plotPoint(targetX[i], targetY[i], (i + 1).toString(), 0x00FF00)
    }
  }

  if (g_Graphics.ShowCalibrationPoints_2 > 0) {
    let targetX = this.myCalibrations.calibrations[2].calibrationData.targetX
    let targetY = this.myCalibrations.calibrations[2].calibrationData.targetY

    for (let i = 0; i < targetX.length; i++) {
      gr.plotPoint(targetX[i], targetY[i], (i + 1).toString(), 0x00FF00)
    }
  }
  if (g_Graphics.ShowCrossHair != null) {
    let count = g_Graphics.ShowCrossHair.length

    for (let i = 0; i < count; i++) {
      var lines = CrossHair(g_Graphics.ShowCrossHair[i][0], g_Graphics.ShowCrossHair[i][1], g_Graphics.ShowCrossHair[i][2])
      gr.plotLine(lines[0][0], lines[0][1], lines[0][2], lines[0][3], '', colors[i], 1, 0, 0)
      gr.plotLine(lines[1][0], lines[1][1], lines[1][2], lines[1][3], '', colors[i], 1, 0, 0)
    }
  }
  tracer.addMessage('<- Draw ' + timeTracker.getElapsedTime())
}
/**
 * This function checks the 'VeryFirstInitializationDone' bit
 * and if it is not set then all regions will be adjusted to the 
 * current hardware. This happens only at the first time of loading 
 * a new job.
 * */
AS200.prototype.initVeryFirstTime = function () {
  let firstTime = InSightFunctions.fnGetCellValue('Internal.VeryFirstInitializationDone')
  if (firstTime == 0) {
    InSightFunctions.fnSetCellValue('Internal.VeryFirstInitializationDone', 1)
  }
}

/**
 * Each script is executed once during the load process.
 * This function handles the initialization and is only called once. 
 * This is called from inside the normal "run-function" 
 * */
AS200.prototype.firstRun = function () {
  myLog('Script Base: -> First Run')

  this.initVeryFirstTime()

  this.triggerMode = InSightFunctions.fnGetCellValue('TriggerMode')
  this.runStateMachine = this.runSlaveStateMachine
  this.autoExposureState = InSightFunctions.fnGetCellValue('AutoImageAdjustment.Enable')
  
  writeCellValue('Internal.AS200.Version', version.toString())
  writeCellValue('Internal.CogMath.Version', cogMath.version.toString())
  writeCellValue('Internal.CogUtils.Version', cogUtils.version.toString())

  writeCellValue('HECalibration.NewCalibrationDone', 0)
  writeCellValue('Internal.RecipeLoaded', 0)
  writeCellValue('Internal.LoadSlaves', 0)
  writeCellValue('Init.Hostname', '127.0.0.1')
  writeCellValue('Communication.EnableTimer', 0)

  g_HRes = InSightFunctions.fnGetSystemConfig('HRESOLUTION')
  g_VRes = InSightFunctions.fnGetSystemConfig('VRESOLUTION')
  g_Border = [[0, 0, 0, g_HRes],
  [0, g_HRes, g_VRes, g_HRes],
  [g_VRes, g_HRes, g_VRes, 0],
  [g_VRes, 0, 0, 0]]

  asLogger.clearLog()

  g_CurrentFeatures = {}
  for (var f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
    g_CurrentFeatures[f] = new Feature(0, 0, 0, 0)
  }  
  this.loadSensorConfig()

  LoadCameraConfigFile()

  this.fillSLMP_Map()
  this.firstRunDone = 1
  myLog('Script Base: <- First Run')
}

/**
 * Triggers the mechanism for loading the slave jobs
 * */
AS200.prototype.firstTimeOnline = function () {
  myLog('-> First time onlinne')

  writeCellValue('Internal.LoadSlaves', 1)

  InSightFunctions.fnUpdateGui(1)

  this.firstTimeOnlineDone = 1
  myLog('<- First time onlinne')
}

/**
 * This function goes through all commands and creates
 * a lookup table for the SLMP IDs
 * GS -> 1010
 * GV -> 1011
 * ...
 * */
AS200.prototype.fillSLMP_Map = function () {
  myLog('-> Fill SLMP-Map ' + timeTracker.getElapsedTime())
  this.slmpMap = {}
  for (var i in this.commands) {
    if (typeof this.commands[i] === 'function') {
      var o = new (this.commands[i])(1, '1,1,1,1,0,0,0,0,0,0,0')

      if (o.hasOwnProperty('_slmpCode')) {
        var code = o._slmpCode
        this.slmpMap[code] = i
      }
    }
  }
  myLog(this.slmpMap)
  myLog('<- Fill SLMP-Map ' + timeTracker.getElapsedTime())
}

/**
 * This function writes all parameters for the TCP-devices 
 * to the spread sheet. 
 * */
AS200.prototype.initTCPDevices = function () {
  myLog('-> initTCPDevices')
  try {
    if (this.myDetails.iAmMaster == 1) {
      for (var i = 1; i <= 2; i++) {
        let port = INTERNAL_PORT
        let ip = ''
        let enable = 0
        if (this.mySensor.cams.hasOwnProperty('Cam_' + i.toString()) == true) {
          let cam = this.mySensor.cams['Cam_' + i.toString()]
          ip = cam.IPAddress
          if (cam.Master == true) {
            port = this.mySensor.plcPort
            ip = ''
          }
          enable = 1
        }

        let t1 = 'Communication.TCPDevice.' + i.toString() + '.Port'
        writeCellValue(t1, port)

        let t2 = 'Communication.TCPDevice.' + i.toString() + '.HostIP'
        writeCellValue(t2, ip)

        let t3 = 'Communication.TCPDevice.' + i.toString() + '.Enabled'
        writeCellValue(t3, enable)
      }
    } else {
      writeCellValue('Communication.TCPDevice.1.Port', INTERNAL_PORT)
      writeCellValue('Communication.TCPDevice.1.HostIP', '')
      writeCellValue('Communication.TCPDevice.1.Enabled', 1)
      writeCellValue('Communication.TCPDevice.2.Enabled', 0)
    }
  } catch (e) {
    myLog(e)
  } finally {
    myLog('<- initTCPDevices')
  }
}

AS200.prototype.onLogClear = function () {
  tracer.clear()
}

AS200.prototype.onTest = function () {
  console.log("onTest")
  console.log("currState: " + this.currentState)
  console.log(g_LooukupTables.parts[6].Description)
  console.log(g_LooukupTables.parts[6].LCResultMode)
  console.log(g_LooukupTables.parts[6].LCLimits[0])
  console.log(g_LooukupTables.parts[6].LCLimits[1])
  //tracer.addMessage('@@@ ' + timeTracker.getElapsedTime())
  ///tracer.addMessage("TestMessage")
}

AS200.prototype.resetStateMachine = function () {
  this.currentState = WAITING_FOR_NEW_COMMAND;
  console.log("currState: " + this.currentState)
}

AS200.prototype.onTest2 = function (a,b) {
  return a+b
}

/**
 * Clears the log. Is triggered from the "Clear" button
 * in the spread sheet
 * */
AS200.prototype.clearLog = function () {
  asLogger.clearLog()
}
/**
 * Load a default recipe. Triggered by the "Reset to default" button.
 * */
AS200.prototype.setRecipeTablesToDefault = function () {
  g_LooukupTables.setToDefault()
}
/**
 * After a TCP device has received something from the PLC or slave or master
 * this function will be called
 * 
 * @param {any} channel index of the channel (1 or 2)
 * @param {any} plc if this device used for the communication to the PLC 
 * @param {any} master if this device used for the communication to the master
 * @param {any} slave if this device used for the communication to the slave
 * @param {any} timeout this is true if the slave does not send a result within the timeout
 * @param {any} device this is true if the device has received some data
 * @param {any} data the received data as a string
 */
AS200.prototype.onDataReceived = function (channel, plc, master, slave, timeout, device, data) {
  let ret = '---'
  if (device == 1) {
    if (plc == 1) {
      this.onDataFromPLCReceived(channel, data)
      ret = data
    } else if (master == 1) {
      this.onDataFromMasterReceived(channel, data)
      ret = data
    } else if (slave == 1) {
      this.onDataFromSlaveReceived(channel, timeout, data)
      ret = data
    } else {
      myLog('No valid TCP configuration!')
    }
  } else if ((timeout == 1) && (slave == 1)) {
    this.onDataFromSlaveReceived(channel, timeout, data)
    ret = 'Timeout'
    myLog('Timeout!')
  }
  return ret
}
/**
 * This is called from the spread sheet if the sensor has received some data over SLMP
 * 
 * @param {any} data the received data as a string
 */
AS200.prototype.onDataFromPLCOverSLMPReceived = function (data) {
  let split = data.split(',')

  if (this.slmpMap.hasOwnProperty(split[0])) {
    let index = data.indexOf(',')
    data = this.slmpMap[split[0]] + data.substr(index, data.length)
  }

  this.onDataFromPLCReceived(0, data)
}
/**
 * Is called from "onDataReceived"
 * Handles the incoming command from the PLC
 * - checks the current state of the state machine
 * - tries to create a valid command
 * - starts the state machine
 * @param {any} channel index of the channel (1 or 2)
 * @param {any} data the received data as a string
 */
AS200.prototype.onDataFromPLCReceived = function (channel, data) {
  timeTracker.restart()
  tracer.clear()
  data = data.toUpperCase()

  tracer.addMessage('-> Data from PLC received on channel ' + channel + ' -> ' + data)

  asLogger.addLogMessage(0, '-> ' + data)
  if (this.currentState === States.WAITING_FOR_NEW_COMMAND) {
    this.currentCommand = getCommandObject(this.commands, this.myDetails.myIndex, data, this)

    if (typeof this.currentCommand !== 'object') {
      // Unknown command / wrong number of arguments / ....
      let ret = InSightFunctions.fnStringf('%s,%d', data.split(',')[0], this.currentCommand)
      this.sendToPlc(ret)
      this.currentCommand = null
    } else {
      // valid command
      InSightFunctions.fnSetCellValue('Internal.Command', this.currentCommand._splittedCmd[0])
      this.runStateMachine()
    }
  } else {
    // State machine is not waiting for a new command -> busy
    let ret = ''
    if (data == 'RESET') {
      this.currentState = States.WAITING_FOR_NEW_COMMAND
      this.currentCommand = null
      ret = 'RESET,1'
    } else {
      ret = InSightFunctions.fnStringf('%s,%d', data.split(',')[0], ECodes.E_BUSSY)
    }
    this.sendToPlc(ret)
  }
  tracer.addMessage('<- Data from PLC received ' + timeTracker.getElapsedTime())
}
/**
 * Is called from "onDataReceived"
 * Handles the command from the master
 * @param {any} channel index of the channel (1 or 2)
 * @param {any} data the received data as a string
 */
AS200.prototype.onDataFromMasterReceived = function (channel, data) {
  timeTracker.restart()
  tracer.clear()
  data = data.toUpperCase()

  tracer.addMessage('Data from Master received on channel ' + channel + ' -> ' + data)
  if (this.currentState === States.WAITING_FOR_NEW_COMMAND) {
    this.currentCommand = getCommandObject(this.commands, 1, data, this)
    if (typeof this.currentCommand !== 'object') {
      // Unknown command / wrong number of arguments / ....
      let ret = InSightFunctions.fnStringf('%s,%d', data.split(',')[0], this.currentCommand)
      this.sendToMaster(ret)
      this.currentCommand = null
    } else {
      // valid command
      InSightFunctions.fnSetCellValue('Internal.Command', this.currentCommand._splittedCmd[0])
      this.runStateMachine()
    }
  } else {
    // State machine is not waiting for a new command -> bussy
    let ret = InSightFunctions.fnStringf('%s,%d', data.split(',')[0], ECodes.E_BUSSY)
    this.sendToMaster(ret)
  }
}
/**
 * Parses the received result from the slave into the result structure
 * @param {any} channel index of the channel (1 or 2)
 * @param {any} timeout triggered by timeout event 0/1
 * @param {any} data the received data as a string
 */
AS200.prototype.onDataFromSlaveReceived = function (channel, timeout, data) {
  tracer.addMessage('-> Data from Slave received on channel ' + channel + ' -> ' + data)

  let result = new Result()
  result.isNeeded = 1
  result.isValid = 1

  if (timeout != 1) {
    tracer.addMessage('data.length ' + data.length)
    if (data.length > 2) {
      result = JSON.parse(data)
    } else {
      result.state = ECodes.E_UNSPECIFIED
    }
  } else {
    result.state = ECodes.E_TIMEOUT
  }

  this.currentCommand._results[channel] = result

  InSightFunctions.fnSetCellValue('Communication.EnableTimer', 0)
  InSightFunctions.fnUpdateGui(1)
  if (this.currentState == States.WAITING_FOR_SLAVE_RESULT) {
    this.runStateMachine()
  }

  tracer.addMessage('<- Data from Slave received ' + timeTracker.getElapsedTime())
}
/**
 * Clears all trained data
 * This function is used to create the template job and is triggered 
 * by the "Reset to factory settings" button.
 */
AS200.prototype.onResetToFactorySettings = function () {
  myLog('-> Reset to factory settings')

  for (let f in g_TrainedFeatures) {
    g_TrainedFeatures[f].splice(MAX_GRIPPERS)
    for (let g = 0; g < MAX_GRIPPERS; g++) {
      g_TrainedFeatures[f][g] = new Feature(0, 0, 0, 0)
    }
  }

  for (let f in g_TrainedRobotPoses) {    
    g_TrainedRobotPoses[f].splice(MAX_GRIPPERS)
    for (let g = 0; g < MAX_GRIPPERS; g++) {
      g_TrainedRobotPoses[f][g] = new RobotPose(0, 0, 0, 0, 0, 0, 0)
    }
  }

  myLog(g_TrainedFeatures)
  myLog(g_TrainedRobotPoses)
  myLog('<- Reset to factory settings')
}
/**
 * If the user changes the feature type (PatMax or Corner)
 * all trained features must be cleared
 * @param {any} channel feature ID 1-4
 */
AS200.prototype.onFeatureTypeChanged = function (channel) {
  myLog('-> Feature Type changed')
  let features = g_LooukupTables.features
  for (let f in features) {
    if ((features[f]['CameraID'] == this.myDetails.myIndex) && (features[f]['CamFeatureID'] == channel)) {
      for (let g = 0; g < MAX_GRIPPERS; g++) {
        g_TrainedFeatures[f][g].valid = 0
        g_TrainedFeatures[f][g].x = 0
        g_TrainedFeatures[f][g].y = 0
        g_TrainedFeatures[f][g].thetaInDegrees = 0
      }
    }
  }
  myLog('<- Feature Type changed')
}
/**
 * Reloads the sensor configuration file
 * The button on the spread sheet is pressed from the HMI
 * */
AS200.prototype.onReloadSensorConfig = function () {
  myLog('-> Reload SensorConfig file')
  this.loadSensorConfig()
  myLog('<- Reload SensorConfig file')
}
/**
 * Reloads the camera configuration file
 * The button on the spread sheet is pressed from the HMI
 * */
AS200.prototype.onReloadCameraConfig = function () {
  myLog('-> Reload CameraConfig')
  LoadCameraConfigFile()
  myLog('<- Reload CameraConfig')
}
/**
 * Deletes the all HE-calibrations and the intrinsic calibration data 
 * and starts the intrinsic calibration
 * */
AS200.prototype.onStartIntrinsicCalibration = function () {

  tracer.addMessage('-> Start Intrinsic Calibration ' + timeTracker.getElapsedTime())
  cogUtils.deleteFile(/HeCalibrationData.*.json/)
  cogUtils.deleteFile(/IntrinsicCalibrationData.*.cxd/)
  InSightFunctions.fnSetCellValue('IntrinsicCalibration.RunCalibration', 1)
  tracer.addMessage('<- Start Intrinsic Calibration ' + timeTracker.getElapsedTime())
}
/**
 * Resets the start bit for the intrinsic calibration and reloads
 * the HE-calibrations to update the informations in the spread sheet 
 * */
AS200.prototype.onIntrinsicCalibrationDone = function () {

  tracer.addMessage('-> Intrinsic Calibration done!' + timeTracker.getElapsedTime())
  InSightFunctions.fnSetCellValue('IntrinsicCalibration.RunCalibration', 0)
  this.onReloadHeCalibrations()
  tracer.addMessage('<- Intrinsic Calibration done!' + timeTracker.getElapsedTime())
}
/**
 * Deletes the all HE-calibrations and the intrinsic calibration data
 * */
AS200.prototype.onUncalibrateIntrinsicCalibration = function () {

  tracer.addMessage('-> Uncalibrate Intrinsic Calibration!' + timeTracker.getElapsedTime())
  cogUtils.deleteFile(/HeCalibrationData.*.json/)
  cogUtils.deleteFile(/IntrinsicCalibrationData.*.cxd/)
  InSightFunctions.fnSetCellValue('IntrinsicCalibration.RunImport', 1)
  tracer.addMessage('<- Uncalibrate Intrinsic Calibration!' + timeTracker.getElapsedTime())
}
/**
 * Reloads the HE-calibration after the intrinsic calibration is loaded
 * */
AS200.prototype.onImportCalibrationDataDone = function () {
  tracer.addMessage('-> Import Intrinsic Calibration Data done!' + timeTracker.getElapsedTime())
  let b = InSightFunctions.fnGetCellValue('IntrinsicCalibration.RunImport')
  if (b == 1) {
    InSightFunctions.fnSetCellValue('IntrinsicCalibration.RunImport', 0)
    this.onReloadHeCalibrations()
  }
  tracer.addMessage('<- Import Intrinsic Calibration Data done!' + timeTracker.getElapsedTime())
}
/**
 * Reloads the HE-calibration
 * */
AS200.prototype.onReloadHeCalibrations = function () {
  tracer.addMessage('-> Reload HE Calibration Data!' + timeTracker.getElapsedTime())
  this.myCalibrations = new Calibrations()
  this.myCalibrations.loadCalibrationsFromFile()
  g_Calibrations = this.myCalibrations.calibrations
  tracer.addMessage('<- Reload HE Calibration Data!' + timeTracker.getElapsedTime())
}
/**
 * Sends the result string to the PLC
 * Writes the result string to the spread sheet and 
 * fires the "Send to PLC" event (event 82) 
 * and triggers the update of the logger with the event 86
 * @param {any} resultStr 
 */
AS200.prototype.sendToPlc = function (resultStr) {
  tracer.addMessage('-> Send to PLC ' + timeTracker.getElapsedTime())
  asLogger.addLogMessage(5, '<- ' + resultStr)
  InSightFunctions.fnSetCellValue('Communication.TCPDevice.SendingToPLC', resultStr)
  InSightFunctions.fnSetEvent(82)
  InSightFunctions.fnSetEvent(86)

  tracer.addMessage('<- Send to PLC ' + timeTracker.getElapsedTime())
}
/**
 * Sends the result string to the master
 * Writes the result string to the spread sheet and
 * fires the "Send to Master/Slave" event (event 81)
 * and triggers the update of the logger with the event 86
 * @param {any} resultStr
 */
AS200.prototype.sendToMaster = function (resultStr) {
  tracer.addMessage('-> Send to Master ' + timeTracker.getElapsedTime())
  asLogger.addLogMessage(0, '-> ' + resultStr)
  InSightFunctions.fnSetCellValue('Communication.TCPDevice.SendingToMaster', resultStr)
  InSightFunctions.fnSetEvent(81)
  InSightFunctions.fnSetEvent(86)
  tracer.addMessage('<- Send toMaster ' + timeTracker.getElapsedTime())
}
/**
 * Sends the command to the slave
 * Writes the command to spread sheet, starts the timer
 * and fires the event 81 ("Send to Master/Slave")
 * @param {any} cmdString
 */
AS200.prototype.sendToSlaves = function (cmdString) {
  tracer.addMessage('-> Send command to slave ' + timeTracker.getElapsedTime())

  let slaveCommand = this.currentCommand.getSlaveCommandString()
  InSightFunctions.fnSetCellValue('Communication.TCPDevice.SendingToSlave', slaveCommand)
  InSightFunctions.fnSetCellValue('Communication.EnableTimer', 1)
  InSightFunctions.fnSetEvent(81)

  tracer.addMessage('<- Send command to slave ' + timeTracker.getElapsedTime())
}
/**
 * The autoExposureState shows if the "set auto exposure time" from the 
 * HMI is running.
 * @param {any} state 0=disabled, 1= enabled
 */
AS200.prototype.setAutoExposureState = function (state) {
  this.autoExposureState = state
}
/**
 * This function creates the feature mask if a feature
 * is enabled/disable from the HMI 
 * This function is also used to create the feature mask during runtime.
 * @param {any} feature_1
 * @param {any} feature_2
 * @param {any} feature_3
 * @param {any} feature_4
 * @param {any} inspection
 */
AS200.prototype.setManualFeatureMask = function (feature_1, feature_2, feature_3, feature_4, inspection) {
  this.currentFeatureMask = feature_1 + ((!!feature_2) << 1) + ((!!feature_3) << 2) + ((!!feature_4) << 3) + (inspection << 8)
}
/**
 * This function returns the feature mask during runtime after an image was acquired
 * to enable/disable the features.
 */
AS200.prototype.imageAcquired = function () {
  tracer.addMessage('-> Image acquired ' + timeTracker.getElapsedTime())
 
  let feature_1=0
  let feature_2=0
  let feature_3=0
  let feature_4=0
  let inspection=0
  
  for(var i = 0; i<arguments.length-1;i++){
    let ii=i+1
    eval("feature_"+ii+"=arguments[i]")
  }
  
  inspection = arguments[arguments.length-1]

  let mask = 1
  g_Graphics.ShowCalibrationPoints_1 = false
  g_Graphics.ShowCalibrationPoints_2 = false

  g_Graphics.ShowCrossHair = null

  if (this.currentState == States.WAITING_FOR_IMAGE_ACQUIRED) {
    InSightFunctions.fnSetCellValue('HECalibration.2.ShowNotValid', 0)

    if (this.autoExposureState == false) {
      this.runStateMachine()
      this.currentFeatureMask = this.currentCommand._enabledFeatures
      mask = this.currentFeatureMask
    } else {
      mask = 0
    }
  } else {
    this.setManualFeatureMask(feature_1, feature_2, feature_3, feature_4, inspection)
    mask = this.currentFeatureMask
  }
  tracer.addMessage('<- Image acquired ' + timeTracker.getElapsedTime())
  return mask
}
/**
 * This function registers an inspection
 * @param {any} index the ID of the inspection
 */
AS200.prototype.addInspection = function (index) {
  let result = false
  if (!g_Inspections.hasOwnProperty(index)) {
    g_Inspections[index] = new Inspection(index)
    result = true
  }
  return result
}
/**
 * This function removes an registered inspection from the inspection list
 * @param {any} index the ID of the inspection
 */
AS200.prototype.removeInspection = function (index) {
  let result = false
  if (g_Inspections.hasOwnProperty(index)) {
    delete g_Inspections[index]
    result = true
  }
  return result
}
/**
 * 
 * @param {any} index
 * @param {any} userFunction
 */
AS200.prototype.registerNewComputeTotalResult = function (index, userFunction) {
  if (!g_Inspections.hasOwnProperty(index)) {
    g_Inspections[index] = new Inspection(index)
  }

  g_Inspections[index].userComputeTotalResult = userFunction
}
/**
 * For future
 * @param {any} index
 * @param {any} isCam1Used
 * @param {any} isCam2Used
 * @param {any} isPartMoving
 * @param {any} shuttlingPos
 */
AS200.prototype.onInspectionSettingsChanged = function (index, isCam1Used, isCam2Used, isPartMoving, shuttlingPos) {

}

/**
 * This function updates the internal acquisition settings for an inspection and writes the new values to the spread sheet
 * @param {any} index
 * @param {any} exposure
 * @param {any} mode
 * @param {any} light1
 * @param {any} light2
 * @param {any} light3
 * @param {any} light4
 * @param {any} enabled
 */
AS200.prototype.onInspectionAcqSettingsChanged = function (index, exposure, mode, light1, light2, light3, light4, enabled) {

  if (!g_Inspections.hasOwnProperty(index)) {
    this.addInspection(index)
  }
  if (enabled == null) {
    enabled = true
  }

  g_Inspections[index].acqSettings.exposure = exposure
  g_Inspections[index].acqSettings.mode = mode
  g_Inspections[index].acqSettings.light1 = light1
  g_Inspections[index].acqSettings.light2 = light2
  g_Inspections[index].acqSettings.light3 = light3
  g_Inspections[index].acqSettings.light4 = light4

  setInspectionAcqSettings(index, enabled)
}
/**
 * Updates the settings for the inspection on the spread sheet
 * @param {any} index
 * @param {any} enabled
 */
AS200.prototype.onInspectionSelectionChanged = function (index, enabled) {
  setInspectionAcqSettings(index, enabled)
}
/**
 * Copies the result data into the result buffer of the current command
 * and runs the state machine
 * @param {any} index index of the inspection
 * @param {any} state state of the command
 * @param {any} data result data
 */
AS200.prototype.onInspectionToolsDone = function (index, state, data) {
  tracer.addMessage('-> Inspection Tools done ' + timeTracker.getElapsedTime() + ' <State= ' + this.currentState + '>')
  if (this.currentCommand != null) {
    this.currentCommand._results[this.currentCommand._myIndex].isValid = 1
    this.currentCommand._results[this.currentCommand._myIndex].state = state
    this.currentCommand._results[this.currentCommand._myIndex].data.push(data)

    this.runStateMachine()
  }
  tracer.addMessage('<- Inspection Tools done ' + timeTracker.getElapsedTime())
}
/**
 * Returns the part data as an JSON object
 * (can be used in the inspection scripts)
 * @param {any} partID ID of the part
 */
AS200.prototype.getPartData = function (partID) {
  let retObj = ""
  if (g_Parts.hasOwnProperty(partID)) {
    retObj = JSON.stringify(g_Parts[partID])
  }

  return retObj
}
/**
 * Transforms the current pixel coordinates into world coordinates
 * 
 * This function was created as the "bridge" between the spread sheet scripts 
 * and the internal function to have a possibility to access the internal transformation
 * (can be used in the inspection scripts)
 * 
 * @param {any} shuttlingPoseIndex
 * @param {any} partIsMoving
 * @param {any} featureX
 * @param {any} featureY
 * @param {any} featureAngle
 * @param {any} featureValid
 * @param {any} robX
 * @param {any} robY
 * @param {any} robTheta
 */
AS200.prototype.inspectionGetTransformed = function (shuttlingPoseIndex, partIsMoving, featureX, featureY, featureAngle, featureValid, robX, robY, robTheta) {
  let feature = new Feature(featureX, featureY, featureAngle, featureValid)
  let robot = new RobotPose(robX, robY, 0, robTheta, 0, 0, 1)
  let isCameraMoving = false
  if (g_Calibrations[shuttlingPoseIndex].calibration !== null) {
    isCameraMoving = g_Calibrations[shuttlingPoseIndex].calibration.isCameraMoving_
  }

  let res = getTransformed(g_Calibrations, shuttlingPoseIndex, isCameraMoving, partIsMoving, feature, robot)

  return res
}
/**
 * This function copies all results from the spread sheet into the internal
 * g_CurrentFeature buffer.
 * This function does not longer need the arguments as single arguments,
 * it will use the arguments array of the function.
 * Now it is easy to add more features without changing the code.
 * The order of the arguments are important it must be the same as you can see below.
 * 
 * @param {any} enable_1 is the feature enabled
 * @param {any} trained_1 is the feature (PatMax) trained
 * @param {any} valid_1 is the result valid (the feature/edge/intersection point)
 * @param {any} x_1 x value
 * @param {any} y_1 y value
 * @param {any} angle_1 angle
 * @param {any} enable_2 
 * @param {any} trained_2
 * @param {any} valid_2
 * @param {any} x_2
 * @param {any} y_2
 * @param {any} angle_2
 * @param {any} enable_3
 * @param {any} trained_3
 * @param {any} valid_3
 * @param {any} x_3
 * @param {any} y_3
 * @param {any} angle_3
 * @param {any} enable_4
 * @param {any} trained_4
 * @param {any} valid_4
 * @param {any} x_4
 * @param {any} y_4
 * @param {any} angle_4
 */
AS200.prototype.toolsDone = function (enable_1, trained_1, valid_1, x_1, y_1, angle_1, 
                                      enable_2, trained_2, valid_2, x_2, y_2, angle_2,
                                      enable_3, trained_3, valid_3, x_3, y_3, angle_3,
                                      enable_4, trained_4, valid_4, x_4, y_4, angle_4) {
  tracer.addMessage('-> Tools done ' + timeTracker.getElapsedTime() + ' <State= ' + this.currentState + '>')

  let count = arguments.length/6
  if (arguments.length % 6 != 0) {
    myLog('#########################################')
    myLog('Error in AS200.prototype.toolsDone')
    myLog('#########################################')

    for (let f = 1; f <= count; f++) {    
        g_CurrentFeatures[f] = new Feature(0, 0, 0, 0)    
    }
  } else {
    for (let f = 1; f <= count; f++) {
      let argOff = (f - 1) * 6

      if (arguments[argOff + 0] > 0) {
        g_CurrentFeatures[f] = new Feature(arguments[argOff + 3], arguments[argOff + 4], arguments[argOff + 5], arguments[argOff + 2])
        if (arguments[argOff + 2] <= 0) {
          if (arguments[argOff + 1] == 0) {
            g_CurrentFeatures[f].valid = ECodes.E_FEATURE_NOT_TRAINED
          } else {
            g_CurrentFeatures[f].valid = ECodes.E_FEATURE_NOT_FOUND
          }
        }
      } else {
        g_CurrentFeatures[f] = new Feature(0, 0, 0, 0)
      }
    }
  }

  this.runStateMachine()

  tracer.addMessage('<- Tools done ' + timeTracker.getElapsedTime())
}
/**
 * Returns the feature mask which features should be enabled for
 * manual align
 * Checks which feature was enabled but failed during the last command.
 * @param {any} enable not used, was for future
 */
AS200.prototype.enableManualSetFeaturePose = function (enable) {  
  let mask = this.currentFeatureMask
  let editFeatureManu = []

  for (var i = 1; i <= 4; i++) {
    let e = (mask >> (i - 1) & 0x01)    
    let v = e ^ (g_CurrentFeatures[i]['valid']!=1 ? 0:1 )    
    editFeatureManu.push(v)    
  }
  return editFeatureManu
}
/**
 * Returns the feature mask which features where enabled during the last command
 * @param {any} enable not used, was for future 
 */
AS200.prototype.getEnabledFeatures = function (enable) {
  let mask = this.currentFeatureMask
  let enabledFeatures = []

  for (var i = 1; i <= 4; i++) {
    let e = (mask >> (i - 1) & 0x01)    
    enabledFeatures.push(e)
  }
  return enabledFeatures
}
/**
 * Replace the position of the selected feature with the new position 
 * @param {any} featureIndex
 * @param {any} newX
 * @param {any} newY
 * @param {any} newAngle
 */
AS200.prototype.setNewFeaturePosition = function (featureIndex, newX, newY, newAngle) {  
  g_CurrentFeatures[featureIndex] = new Feature(newX, newY, newAngle, 1)
}
/**
 * Restarts the state machine with the last command from the 
 * toolsDone state 
 * (computes the new final result and sends the result back to the PLC)
 * */
AS200.prototype.restartLastCmdFromToolsDone = function () {  
  this.currentCommand = this.lastCommand
  this.currentCommand._results[this.currentCommand._myIndex] = new Result()
  this.currentCommand._results.error = ECodes.E_NO_ERROR
  this.currentState = States.WAITING_FOR_TOOLS_DONE
  this.runStateMachine()  
}
/**
 * Updates the spread sheet with the informations from the calibrations
 * */
AS200.prototype.writeCalibrationInfosToSheet = function () {
  if (this.myCalibrations.calibrations['1'] != null) {
    let calib = this.myCalibrations.calibrations['1']
    InSightFunctions.fnSetCellValue('HECalibration.Valid', calib.runstatus)

    if (calib.runstatus == 1) {
      let trans = new cogMath.cc2XformLinear()
      if (calib['calibration']['isCameraMoving_'] == true) {
        trans.setXform(calib.results.Transforms.Stage2DFromImage2D.xform)
      } else {
        trans.setXform(calib.results.Transforms.Home2DFromImage2D.xform)
      }
      let xScale = trans.xScale()
      let yScale = trans.yScale()

      let diagnostics = calib.results.Diagnostics
      writeCellValue('HECalibration.MaxImage2D', diagnostics['OverallResidualsImage2D']['Max'])
      writeCellValue('HECalibration.RMSImage2D', diagnostics['OverallResidualsImage2D']['Rms'])
      writeCellValue('HECalibration.MaxHome2D', diagnostics['OverallResidualsHome2D']['Max'])
      writeCellValue('HECalibration.RMSHome2D', diagnostics['OverallResidualsHome2D']['Rms'])
      writeCellValue('HECalibration.PixelSizeX', xScale)
      writeCellValue('HECalibration.PixelSizeY', yScale)
    }
  } else {
    writeCellValue('HECalibration.Valid', 0)
    writeCellValue('HECalibration.MaxImage2D', -1)
    writeCellValue('HECalibration.RMSImage2D', -1)
    writeCellValue('HECalibration.MaxHome2D', -1)
    writeCellValue('HECalibration.RMSHome2D', -1)
    writeCellValue('HECalibration.PixelSizeX', 0)
    writeCellValue('HECalibration.PixelSizeY', 0)
  }
}
/**
 * Loads the sensor configuration file and initializes the 
 * look up tables
 * */
AS200.prototype.loadSensorConfig = function () {
  myLog('-> Load SensorConfig')
  // Try to load the SensorConfiguration
  this.mySensor = new Sensor()
  let sensorConfigFile = cogUtils.loadFile(SENSORCONFIG_FILENAME)

  if (Object.keys(sensorConfigFile).length > 0) {
    this.mySensorConfiguration = new SensorConfiguration()
    this.mySensorConfiguration.init(sensorConfigFile)

    this.mySensor.initFromSensorConfig(this.mySensorConfiguration)
    
    this.myRecipes = new Recipes(this.mySensorConfiguration)

    this.myDetails = new MyDetails()
    this.myDetails.myIndex = this.mySensor.findIndexByIp(this.myDetails.myIP)
    this.myDetails.iAmMaster = 1 

    let usedCameras = this.myRecipes.recipes[(this.myDetails.jobName).toLowerCase()]['UsedCameras']    
    g_LooukupTables = new RecipeTables(this.myDetails.myIndex, usedCameras)
    g_LooukupTables.readFromSheet()

    this.commands = masterCommands

    this.runStateMachine = this.runMasterStateMachine
  } else {
    this.myDetails = new MyDetails()
    this.myDetails.myIndex = 0
    this.myDetails.iAmMaster = 0
    this.runStateMachine = this.runSlaveStateMachine

    g_TrainedRobotPoses = {}
    g_TrainedFeatures = {}
    for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
      if ((g_StoredTrainedFeatures != null) && (g_StoredTrainedFeatures.hasOwnProperty(f))) {
        g_TrainedFeatures[f] = g_StoredTrainedFeatures[f]
      } else {
        g_TrainedFeatures[f] = []
        for (let g = 0; g < MAX_GRIPPERS; g++) {
          g_TrainedFeatures[f][g] = new Feature(0, 0, 0, 0)
        }
      }
    }

    this.commands = slaveCommands
  }
  this.commands.myIndex = this.myDetails.myIndex

  this.myCalibrations = new Calibrations()
  this.myCalibrations.loadCalibrationsFromFile()
  g_Calibrations = this.myCalibrations.calibrations



  writeCellValue('HECalibration.Selector', 0)
  this.mySensor.writeToSheet()
  this.myDetails.writeToSheet()
  this.initTCPDevices()
  myLog('<- Load SensorConfig')
}

//* ***************************************************************************/

AS200.prototype.getCamerIDFromFeatureID = function (featureID) {
  return g_LooukupTables.features[featureID]['CameraID']
  
}
AS200.prototype.getCamerFeatureIDFromFeatureID = function (featureID) {
  return g_LooukupTables.features[featureID]['CamFeatureID']
}

AS200.prototype.writeLogToSheet = function () {
  let ftpMessageString = asLogger.writeLogToSheet()
  return ftpMessageString
}
//* ***************************************************************************/
//-> Print functions for "debugging" 
AS200.prototype.onPrintCalibrations = function () {
  myLog('-> g_Calibrations')
  myLog(g_Calibrations)
  myLog('<- g_Calibrations')
}
AS200.prototype.onPrintRuntimeFeatures = function () {
  myLog('-> g_RuntimeFeatures')
  myLog(g_RuntimeFeatures)
  myLog('<- g_RuntimeFeatures')
}
AS200.prototype.onPrintTrainedFeatures = function () {
  myLog('-> g_TrainedFeatures')
  myLog(g_TrainedFeatures)
  myLog('<- g_TrainedFeatures')
}
AS200.prototype.onPrintTrainedRobotPoses = function () {
  myLog('-> g_TrainedRobotPoses')
  myLog(g_TrainedRobotPoses)
  myLog('<- g_TrainedRobotPoses')
}
AS200.prototype.onPrintCurrentFeatures = function () {
  myLog('-> g_CurrentFeatures')
  myLog(g_CurrentFeatures)
  myLog('<- g_CurrentFeatures')
}
AS200.prototype.onPrintParts = function () {
  myLog('-> g_Parts')
  myLog(g_Parts)
  myLog('<- g_Parts')
}
AS200.prototype.onPrintFeaturesInfos = function () {
  myLog('-> g_FeaturesInfos')
  myLog(g_FeaturesInfos)
  myLog('<- g_FeaturesInfos')
}
AS200.prototype.onPrintStepsByID = function () {
  myLog('-> g_Steps')
  myLog(g_Steps)
  myLog('<- g_Steps')
}
AS200.prototype.onPrintRecipeTables = function () {
  myLog('-> g_LooukupTables')
  myLog(g_LooukupTables)
  myLog('<- g_LooukupTables')
}
AS200.prototype.onPrintTrace = function () {
  myLog('-> Trace')
  tracer.print()
  myLog('<- Trace')
}
//<- Print functions for "debugging" 
//* ***************************************************************************/

/**
 * This function marks the needed features of each camera in the 
 * result structure.
 * This is needed to check if all results are received.
 * */
AS200.prototype.newValidCommandReceived = function () {
  this.lastCommand = this.currentCommand
  if ((this.currentCommand._sendToSlave > 0) && (this.currentCommand._onlyForMaster == 0)) {
    this.sendToSlaves('cmdString')
  }
  let index = this.currentCommand._index
  if (this.currentCommand._useAsStepID == 1) {
    for (let i = 1; i <= MAX_CAMERAS; i++) {
      this.currentCommand._results[i].isNeeded = g_LooukupTables.stepLookup[index]['Cam_' + i].Enabled && (!this.currentCommand._onlyForMaster)
      this.currentCommand._results[i].isValid = 0
    }
  } else if (this.currentCommand._useAsPartID == 1) {
    for (let i = 1; i <= MAX_CAMERAS; i++) {
      this.currentCommand._results[i].isNeeded = g_LooukupTables.partLookup[this.currentCommand._partID1]['Cam_' + i].Enabled && (!this.currentCommand._onlyForMaster)
      this.currentCommand._results[i].isValid = 0
    }
  } else {
    if ((g_LooukupTables.cameraLookup[index]['SendToSlave'] == 1) || (index == this.myDetails.myIndex) || (index == 0)) {
      for (let i = 1; i <= MAX_CAMERAS; i++) {
        if (((index == 0) && (g_LooukupTables.cameraLookup[i]['SendToSlave'] == 1)) ||
          ((index == 0) && (i == this.myDetails.myIndex)) ||
          (index == i)) {
          this.currentCommand._results[i].isNeeded = 1
        }
        this.currentCommand._results[i].isValid = 0
      }
    } else {
      this.currentCommand._results.error = ECodes.E_INVALID_CAMERA_ID
    }
  }
  if ((this.currentCommand._results[this.myDetails.myIndex].isNeeded == 1) || (this.currentCommand._onlyForMaster > 0)) {
    this.currentState = this.currentCommand.execute(this)
  } else {
    this.currentState = States.WAITING_FOR_SLAVE_RESULT
  }
  if (this.currentCommand._results.error == ECodes.E_NO_ERROR) {
    if (this.currentState == States.WAITING_FOR_SLAVE_RESULT) {
      let neededCnt = 0
      let validCnt = 0

      for (let i = 1; i <= MAX_CAMERAS; i++) {
        if (this.currentCommand._results[i].isNeeded > 0) {
          neededCnt++
          if (this.currentCommand._results[i].isValid > 0) {
            validCnt++
          }
        }
      }
      if (neededCnt == validCnt) {
        let resStrPlc = this.currentCommand.computeTotalResult()
        this.sendToPlc(resStrPlc)

        this.currentCommand = null
        this.currentState = States.WAITING_FOR_NEW_COMMAND
      }
    }
  } else {
    let plcString = this.currentCommand._splittedCmd[0] + ',' + this.currentCommand._results.error
    this.sendToPlc(plcString)
    this.currentCommand = null
    this.currentState = States.WAITING_FOR_NEW_COMMAND
  }
}
/**
 * State machine for master
 * */
AS200.prototype.runMasterStateMachine = function () {
  tracer.addMessage('-> State machine ' + timeTracker.getElapsedTime())
  tracer.addMessage('-> State = ' + this.currentState)

  switch (this.currentState) {
    case States.WAITING_FOR_NEW_COMMAND:
      if (this.currentCommand != null) {
        this.newValidCommandReceived()
      } else {
        tracer.addMessage('WAITING_FOR_NEW_COMMAND <no valid command>!')
      }
      break

    case States.WAITING_FOR_IMAGE_ACQUIRED:
      if (this.currentCommand != null) {
        this.currentState = this.currentCommand.imgAcquired(this)

        if (this.currentCommand._results.error != ECodes.E_NO_ERROR) {
          let plcString = this.currentCommand._splittedCmd[0] + ',' + this.currentCommand._results.error
          this.sendToPlc(plcString)

          this.currentCommand = null
          this.currentState = States.WAITING_FOR_NEW_COMMAND
        }
      }
      break

    case States.WAITING_FOR_TOOLS_DONE:
      if (this.currentCommand != null) {
        this.currentState = this.currentCommand.toolsDone(this)
        if (this.currentCommand._results.error == ECodes.E_NO_ERROR) {
          if (this.currentState == States.WAITING_FOR_SLAVE_RESULT) {
            let neededCnt = 0
            let validCnt = 0

            for (var i = 1; i <= MAX_CAMERAS; i++) {
              if (this.currentCommand._results[i].isNeeded > 0) {
                neededCnt++
                if (this.currentCommand._results[i].isValid > 0) {
                  validCnt++
                }
              }
            }
            if (neededCnt == validCnt) {
              let resStrPlc = this.currentCommand.computeTotalResult()
              tracer.addMessage(resStrPlc)
              if (resStrPlc === 'REDO_FROM_EXECUTE' ) {
                this.newValidCommandReceived()                
              }
              else {
                this.sendToPlc(resStrPlc)
                this.currentCommand = null
                this.currentState = States.WAITING_FOR_NEW_COMMAND
              }
            }
          }
        } else {
          let plcString = this.currentCommand._splittedCmd[0] + ',' + this.currentCommand._results.error
          this.sendToPlc(plcString)
          this.currentCommand = null
          this.currentState = States.WAITING_FOR_NEW_COMMAND
        }
      }
      break

    case States.WAITING_FOR_SLAVE_RESULT:
      if (this.currentCommand != null) {
        let resStrPlc = this.currentCommand.computeTotalResult()
        tracer.addMessage(resStrPlc)
        if (resStrPlc === 'REDO_FROM_EXECUTE') {
          this.newValidCommandReceived()
        } else {
          this.sendToPlc(resStrPlc)

          this.currentCommand = null
          this.currentState = States.WAITING_FOR_NEW_COMMAND
        }
      }
      break
    default:
  }
    
  tracer.addMessage('<- State = ' + this.currentState)
  tracer.addMessage('<- State machine ' + timeTracker.getElapsedTime())
}
/**
 * State machine for slave
 * */
AS200.prototype.runSlaveStateMachine = function () {
  tracer.addMessage('-> Slave State machine ' + timeTracker.getElapsedTime())
  tracer.addMessage('-> State = ' + this.currentState)

  switch (this.currentState) {
    case States.WAITING_FOR_NEW_COMMAND:
      if (this.currentCommand != null) {
        this.currentState = this.currentCommand.execute(this)

        if (this.currentCommand._results.error == ECodes.E_NO_ERROR) {
          if (this.currentState == States.WAITING_FOR_NEW_COMMAND) {            
            let slaveString = JSON.stringify(this.currentCommand._results['1'])
            this.sendToMaster(slaveString)
            this.currentCommand = null
            this.currentState = States.WAITING_FOR_NEW_COMMAND
          }
        } else {          
          let slaveString = JSON.stringify(this.currentCommand._results['1'])
          this.sendToMaster(slaveString)
          this.currentCommand = null
          this.currentState = States.WAITING_FOR_NEW_COMMAND
        }
      }
      break

    case States.WAITING_FOR_IMAGE_ACQUIRED:
      if (this.currentCommand != null) {
        this.currentState = this.currentCommand.imgAcquired(this)
      }
      break

    case States.WAITING_FOR_TOOLS_DONE:
      if (this.currentCommand != null) {
        this.currentState = this.currentCommand.toolsDone(this)
        let resString = JSON.stringify(this.currentCommand._results['1'])

        this.sendToMaster(resString)
        this.currentCommand = null
        this.currentState = States.WAITING_FOR_NEW_COMMAND
      }
      break
    default:
  }

  if (this.currentState == States.WAITING_FOR_NEW_COMMAND) {
    // Start Looger
    InSightFunctions.fnSetEvent(86)
  }

  tracer.addMessage('<- Slave State = ' + this.currentState)

  tracer.addMessage('<- Slave State machine ' + timeTracker.getElapsedTime())
}
//* ***************************************************************************/
//-> Everything for inspections
/**
 * Object to hold the AcqSettings for a inspection
 * @param {any} exposure
 * @param {any} mode
 * @param {any} light1
 * @param {any} light2
 * @param {any} light3
 * @param {any} light4
 */
function InspectionAcqSettings(exposure, mode, light1, light2, light3, light4) {
  this.exposure = exposure
  this.mode = mode
  this.light1 = light1
  this.light2 = light2
  this.light3 = light3
  this.light4 = light4
}
/**
 * The internal inspection object.
 * This will be created if an inspection is registered 
 * @param {any} index index of the inspection
 */
function Inspection(index) {
  this.acqSettings = new InspectionAcqSettings(0.1, 0, 50, 0, 0, 0)
  this.userComputeTotalResult = null
  this.index = index
}
//<- Everything for inspections
//* ***************************************************************************/


//* ***************************************************************************/
// Commands MASTER
//* ***************************************************************************/
function CmdInfos() {
  this.featureMask = 0 // As Bit Array
  this.exposureSet = 1
  this.shuttlingPose = 1
  this.isCameraMoving = 0
  this.isPartMoving = 0
}
/**
 * All master commands are derived from this base
 * @param {any} myIndex
 * @param {any} cmdString
 */
function CommandBase(myIndex, cmdString) {
  this._myIndex = myIndex // index of the camera
  this._cmdString = cmdString // the received command string
  this._splittedCmd = cmdString.split(',') // the split command

// remove whitespace
  let cmdLen = this._splittedCmd.length
  for(let i=0; i< cmdLen;i++){
    this._splittedCmd[i]=this._splittedCmd[i].trim()
  }

  this._hasIndex = 0  // indicates that arg[0] is used as index
  this._useAsStepID = 0 // the index is used to select the correct step 
  this._useAsPartID = 0 // the index is used to select the correct Part
  this._numArguments = -1 // number of min. arguments
  this._slmpCode = -1 // SLPM code
  this._onlyForMaster = 0 // the command does not need any information from the slave
  
  this._validCMD = 0 // the received command is valid

  this._index = -1  // the index from arg[0]
  this._sendToSlave = 0 // 
  this._isCameraMoving = 0
  this._alignMode = 1
  this._enabledFeatures = 0
  this._featureMask = 0
  this._exposureSet = 1
  this._shuttlingPose = 1
  this._isPartMoving = 0
  this._logImageType = ''

  this._partID1 = 0 
  this._partID2 = 0
  this._gripperID1 = 0
  this._computeBothParts = 0


  // this array contains all IDs encoded in the index
  // [0] = the current index-pointer
  // [1] = received & 0x0000 000f
  // [2] = received & 0x0000 00f0
  // ....

  this._decodedIDs = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

  this._slaveCmdInfos = new CmdInfos()

  this._robotPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)
  this.resetResultStructure()
  
};
/**
 * Clears the Result structure. 
 * Is also used to create a new one during creating the command object
 * */
CommandBase.prototype.resetResultStructure = function () {
  this._results = {}
  this._results.error = ECodes.E_NO_ERROR

  for (let i = 1; i <= MAX_CAMERAS; i++) {
    this._results[i] = new Result()
  }
}

CommandBase.prototype.isValid = function () {
  return this._validCMD
}
/**
 *  Is used to check if a special feature is enabled
 * @param {any} featureID the feature ID to check
 */
CommandBase.prototype.isFeatureEnabled = function (featureID) {
  return !!(this._enabledFeatures & (1 << (featureID - 1)))
}
/**
 * The base execute function.
 * Disables all features, sets the acq settings and starts the acq
 * @param {any} t
 */
CommandBase.prototype.execute = function (t) {
  tracer.addMessage('-> Execute ' + timeTracker.getElapsedTime())

  if (this._useAsStepID > 0) {
    if (g_LooukupTables.stepLookup[this._index]['Cam_' + this._myIndex]['Enabled'] == 0) {
      tracer.addMessage('------------------------------------ Error ')
    }
  }

  if (this._useAsPartID > 0) {
    if (g_LooukupTables.partLookup[this._partID1]['Cam_' + this._myIndex]['Enabled'] == 0) {
      tracer.addMessage('------------------------------------ Error ')
    }
  }
  this._enabledFeatures = 0

  setFeatureAcqSettings(this._exposureSet - 1)

  if (this._logImageType.length > 2) {
    InSightFunctions.fnSetCellValue(this._logImageType, 1)
  }

  if (t.triggerMode == 32) {
    InSightFunctions.fnSetEvent(32)
  }

  tracer.addMessage('<- Execute ' + timeTracker.getElapsedTime())
  return States.WAITING_FOR_IMAGE_ACQUIRED
}
/**
 * The base imgAcquired function.
 * Enables the used features
 * */
CommandBase.prototype.imgAcquired = function () {
  tracer.addMessage('-> Image acquired (CMD)' + timeTracker.getElapsedTime())
  this._enabledFeatures = this._featureMask
  tracer.addMessage('<- Image acquired (CMD)' + timeTracker.getElapsedTime())

  return States.WAITING_FOR_TOOLS_DONE
}
/**
 * The base toolsDone function
 * Transforms all feature positions int world coordinates
 * and copies the result into the internal result structure
 * @param {any} t
 */
CommandBase.prototype.toolsDone = function (t) {
  tracer.addMessage('-> Tools done ' + timeTracker.getElapsedTime())
  if (this._logImageType.length > 2) {
    InSightFunctions.fnSetCellValue(this._logImageType, 0)
  }

  this._results[this._myIndex]['isValid'] = 1
  for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
    let transformed = new Feature(0, 0, 0, 0)

    if (this.isFeatureEnabled(f) == true) {
      if (g_CurrentFeatures[f].valid > 0) {
        transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
        if (this._results[this._myIndex].state >= 0) {
          this._results[this._myIndex].state = transformed.valid
        }

      } else {
        this._results[this._myIndex].state = g_CurrentFeatures[f].valid
      }

      this._results[this._myIndex].data.push(transformed)
    }
  }

  tracer.addMessage('<- Tools done ' + timeTracker.getElapsedTime())

  return States.WAITING_FOR_SLAVE_RESULT
}
/**
 * The base computeTotalResult function.
 * Checks the state of the result buffer and
 * creates the result string
 * */
CommandBase.prototype.computeTotalResult = function () {
  tracer.addMessage('-> Compute result ' + timeTracker.getElapsedTime())
  let plcString = ''
  tracer.addMessage(this._results)
  this.checkResultsState()

  let retState = ECodes.E_UNSPECIFIED
  if (this._results.error == ECodes.E_NO_ERROR) {
    retState = 1
  } else {
    retState = this._results.error
  }

  plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], retState)

  tracer.addMessage('<- Compute result ' + timeTracker.getElapsedTime())
  return plcString
}
/**
 * Returns the command string for the slave
 * */
CommandBase.prototype.getSlaveCommandString = function () {
  tracer.addMessage('-> Get slave command string (Base)')

  let slaveCommand = InSightFunctions.fnStringf('%s,%d,%d,%d,%d,%d',
    this._splittedCmd[0],
    this._slaveCmdInfos.featureMask,
    this._slaveCmdInfos.exposureSet,
    this._slaveCmdInfos.shuttlingPose,
    this._slaveCmdInfos.isCameraMoving,
    this._slaveCmdInfos.isPartMoving
  )

  let commandData = this.getCommandData()
  if (commandData.length > 0) {
    slaveCommand = slaveCommand + ',' + commandData
  }
  tracer.addMessage('<- Get slave command string (Base)')
  return slaveCommand
}
/**
 * Returns only the command data without the command and the index
 * */
CommandBase.prototype.getCommandData = function () {
  let end = this._splittedCmd.length
  let start = this._hasIndex + 1
  let data = this.copyCommandData(start, end)
  return data
}
/**
 * 
 * @param {any} start
 * @param {any} end
 */
CommandBase.prototype.copyCommandData = function (start, end) {
  let data = ''
  data = this._splittedCmd.splice(start, end-start).toString()
  /*for (let i = start; i < end; i++) {
    data = data + this._splittedCmd[i] + ','
  }
  data = data.slice(0, data.length - 1)
  */
  return data
}
/**
 * Should be replaced with the following functions
 * @param {any} minIndex
 * @param {any} maxIndex
 */
CommandBase.prototype.checkLengthAndSetIndex = function (minIndex, maxIndex) {
  // Check length and extract the index
  let resState = false
  if (this._splittedCmd.length >= (this._numArguments + this._hasIndex + 1)) {
    if (this._hasIndex) {
      if (!isNaN(this._splittedCmd[1])) {
        let index = parseInt(this._splittedCmd[1])
        this._index = index & MASK_CAM_ID
        let id_1 = this._index 
        let id_2 = -1 
        // this._isCameraMoving = (index & MASK_MOVING_CAMERA) >> SHIFT_MOVING_CAMERA

        this._partID1 = (index & MASK_ID_1) >> SHIFT_ID_1
        this._partID2 = (index & MASK_ID_2) >> SHIFT_ID_2
        this._gripperID1 = (index & MASK_GRIPPER_ID_1) >> SHIFT_GRIPPER_ID_1

        if (this._useAsPartID == 1) {
          id_1 = this._partID1
          id_2 = this._partID2
        }

         if ((id_1 >= minIndex) && (id_1 <= maxIndex)) {
          resState = true
         } else {
          this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
        }

      } else {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
      }
    } else {
      resState = true
    }
  } else {
    this._validCMD = ECodes.E_TOO_FEW_ARGUMENTS // Wrong number of arguments
  }
  return resState
}

/********************************************************************************
 * Section with functions to check that the command and arguments are valid
 * The functions should be used in this order
 *******************************************************************************/

/**
 * Checks the number of received arguments.
 * Returns false if there are not enough 
 * arguments and sets the this.validCMD to ECodes.E_TOO_FEW_ARGUMENTS.
 * */
CommandBase.prototype.checkLength = function () {
  // Check length 
  let resState = false
  if (this._splittedCmd.length >= (this._numArguments + this._hasIndex + 1)) {
    resState = true
  } else {
    this._validCMD = ECodes.E_TOO_FEW_ARGUMENTS // Wrong number of arguments
  }
  return resState
}

/**
 * Checks the type of each argument 
 * (The command itself is not included)
 * Compares the type of the argument with the 
 * provided type list 
 * Types are defined in ARG_TYPES
 * @param {any} types an array with the types
 * e.g. [ARG_TYPES.STRING, ARG_TYPE.INT]
 */
CommandBase.prototype.checkArgTypes = function (types) {

  let typeLength = types.length

  for (let i = 0; i < typeLength; i++) {
    let arg = this._splittedCmd[i + 1]
    let arr = null
    let type = types[i]
    if (typeof type == 'object') {
      if (Array.isArray(type)) {
        if (type.length == 2) {
          arr = type[1]
          type = type[0]
        } else {
          this._validCMD = ECodes.E_INVALID_ARG_TYPE_DEFINITION
          return false
        }
      } else {
        this._validCMD = ECodes.E_INVALID_ARG_TYPE_DEFINITION
        return false
      }
    }
    switch (type) {
      case ARG_TYPES.STRING: myLog('===> Check STRING'); break
      case ARG_TYPES.INT: {
        myLog('===> Check INT');
        if (isNaN(arg)) {
          this._validCMD = ECodes.E_INVALID_ARGUMENT
          return false
        }
      } break
      case ARG_TYPES.FLOAT: {
        myLog('===> Check FLOAT');
        if (isNaN(arg)) {
          this._validCMD = ECodes.E_INVALID_ARGUMENT
          return false
        }
    }break      
      case ARG_TYPES.DEF_COORDINATE_SYS: {
        myLog('===> Check DEF_COORDINATE_SYS');
        if (isValidCoordinateSystem(arg, arr) != true) {
          this._validCMD = ECodes.E_INVALID_COORDINATE_SYSTEM
          return false
        }
      } break
      case ARG_TYPES.DEF_RESULT_MODE: {
        myLog('===> Check DEF_RESULT_MODE');
        if (isValidResultMode(arg, arr) != true) {
          this._validCMD = ECodes.E_INVALID_RESULT_MODE
          return false
        }
      }break
      case ARG_TYPES.DEF_LCHECK_RESULT_MODE: {
        myLog('===> Check DEF_LCHECK_RESULT_MODE');
        if (isValidLCheckResultMode(arg, arr) != true) {
          this._validCMD = ECodes.E_INVALID_LCHECK_RESULT_MODE
          return false
        }
      }break
      default: 
        myLog('===> Check default');
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return false
    }
  }
  return true
}
/**
 * Checks if the command is using an index
 * and checks the argument type for the index 
 * (must be possible to parse it into a number).
 * Sets the "this._index"
 * */
CommandBase.prototype.setIndex = function () {
  // Check index  
  if (this._hasIndex) {
    if (isNaN(this._splittedCmd[1])) {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return false
    } else {
      this._index = parseInt(this._splittedCmd[1])    
    }    
  }
  return true
}

/**
 * Decodes all encoded IDs
 * This function creates a array with a legth of 9
 * [0] is used as a pointer to the used ID
 * [1] ID 1
 * [x] ID x
 * [8] ID 8
 * xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx
 *   8    7    6    5    4    3    2    1
 * */
CommandBase.prototype.decodeIDsfromIndex = function () {
  let toDecode = this._index
    this._decodedIDs[0] = 0
    if (toDecode === 0) {
        if (g_LooukupTables.partLookup.hasOwnProperty(1) && g_LooukupTables.partLookup[1].MPs != null) {
            for (let i = 1; i <= g_LooukupTables.partLookup[1].MPs.length; i++) {
                if (i >= MAX_BACKUP_PARTS)
                    break;
                this._decodedIDs[i] = g_LooukupTables.partLookup[1].MPs[i - 1];
            }
        }
    }
    else {
        //if (toDecode < RECIPE_MAX_PARTS) {
        //    if (g_LooukupTables.partLookup.hasOwnProperty(toDecode)) {
        //        this._decodedIDs[1] = toDecode;
        //        for (let i = 2; i <= g_LooukupTables.partLookup[toDecode].MPs.length; i++) {
        //            if (i >= MAX_BACKUP_PARTS)
        //                break;
        //            this._decodedIDs[i] = g_LooukupTables.partLookup[toDecode].MPs[i - 2];
        //        }
        //    }
        //}
        //else {
            for (let i = 1; i <= MAX_BACKUP_PARTS; i++) {
                this._decodedIDs[i] = (toDecode >>> ((i - 1) * 4)) & 0xf
        //    }
        }
    }
}
/**
 * Sets the pointer to the next ID and returns true
 * If there is no next ID the return value is false
 * Is currently used in multi pattern functions 
 * */
CommandBase.prototype.getNextID = function () {

    for (let i = this._decodedIDs[0] + 1; i <= MAX_BACKUP_PARTS; i++) {
    if (this._decodedIDs[i] > 0) {
      this._decodedIDs[0] = i
      return true
    }
  }
  return false
}
/**
 * Checks if the arguments of a command are numbers
 * @param {any} startIndex the first index of the arguments to check
 * @param {any} endIndex the last index of the arguments to check
 */
CommandBase.prototype.checkArgumentsNumbers = function (startIndex, endIndex) {
  for (let i = startIndex; i <= endIndex; i++) {
    if (isNaN(this._splittedCmd[i])) {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return false
    }
  }
  return true
}
/**
 * Copies the robot pose to the robot pose variable of the command
 * @param {any} start start index of the arguments for the robot pose 
 */
CommandBase.prototype.copyRobotPose = function (start) {
  this._robotPose.x = parseFloat(this._splittedCmd[start])
  this._robotPose.y = parseFloat(this._splittedCmd[start + 1])
  this._robotPose.z = parseFloat(this._splittedCmd[start + 2])
  this._robotPose.thetaZ = parseFloat(this._splittedCmd[start + 3])
  this._robotPose.thetaY = parseFloat(this._splittedCmd[start + 4])
  this._robotPose.thetaX = parseFloat(this._splittedCmd[start + 5])
  this._robotPose.valid = 1
}
/**
 * Fills all informations needed in this command
 * - exposure set
 * - feature mask
 * - shuttling part
 * - moving camera
 * and creates the command header for the slave if needed
 * The informations are coming through the part ID.
 **/
CommandBase.prototype.fillCmdInfoWithPartID = function () {
  if (!g_LooukupTables.partLookup.hasOwnProperty(this._partID1)) {
    this._validCMD = ECodes.E_INVALID_PART_ID
    return false
  }

  let index = parseInt(this._splittedCmd[1])
  let isCameraMoving = (index & MASK_MOVING_CAMERA) >> SHIFT_MOVING_CAMERA
  
  let info = g_LooukupTables.partLookup[this._partID1]
  let info2 = info

  if ((this._partID2 > 0) && (this._computeBothParts == 1)) {
    info2 = g_LooukupTables.partLookup[this._partID2]
  }

  for (let c = 1; c <= MAX_CAMERAS; c++) {
    if (c == this._myIndex) {
      this._featureMask = info['Cam_' + c]['FeatureMask']
      this._exposureSet = info['Cam_' + c]['ExpSetting']
      this._shuttlingPose = info['Cam_' + c]['ShuttlePose']
      this._isPartMoving = info['Cam_' + c]['IsMoving']
      if (this._computeBothParts) {
        this._featureMask = this._featureMask | (info2['Cam_' + c]['FeatureMask'] << SHIFT_ID_2)
        this._shuttlingPose = this._shuttlingPose | (info2['Cam_' + c]['ShuttlePose'] << SHIFT_ID_2)
        this._isPartMoving = this._isPartMoving | (info2['Cam_' + c]['IsMoving'] << SHIFT_ID_2)
      }
    } else {
      this._slaveCmdInfos.featureMask = info['Cam_' + c]['FeatureMask']
      this._slaveCmdInfos.exposureSet = info['Cam_' + c]['ExpSetting']
      this._slaveCmdInfos.shuttlingPose = info['Cam_' + c]['ShuttlePose']
      this._slaveCmdInfos.isPartMoving = info['Cam_' + c]['IsMoving']
      if (this._computeBothParts) {
        this._slaveCmdInfos.featureMask = this._slaveCmdInfos.featureMask | (info2['Cam_' + c]['FeatureMask'] << SHIFT_ID_2)
        this._slaveCmdInfos.shuttlingPose = this._slaveCmdInfos.shuttlingPose | (info2['Cam_' + c]['ShuttlePose'] << SHIFT_ID_2)
        this._slaveCmdInfos.isPartMoving = this._slaveCmdInfos.isPartMoving | (info2['Cam_' + c]['IsMoving'] << SHIFT_ID_2)
      }

      if (c === this._partID1) {
        this._slaveCmdInfos.isCameraMoving = isCameraMoving
      } else {
        this._slaveCmdInfos.isCameraMoving = 0
      }
    }
  }

  this._sendToSlave = info['SendToSlave'] | info2['SendToSlave']
  return true
}
/**
 * Same as ...PartID
 * The informations are coming through the step ID.
 * */
CommandBase.prototype.fillCmdInfoWithStepID = function () {
  if (!g_LooukupTables.stepLookup.hasOwnProperty(this._index)) {
    this._validCMD = ECodes.E_INVALID_ARGUMENT
    return false
  }

  let index = parseInt(this._splittedCmd[1])
  let isCameraMoving = (index & MASK_MOVING_CAMERA) >> SHIFT_MOVING_CAMERA

  let info = g_LooukupTables.stepLookup[this._index]

  for (let c = 1; c <= MAX_CAMERAS; c++) {
    if (c == this._myIndex) {
      this._featureMask = info['Cam_' + c]['FeatureMask']
      this._exposureSet = info['Cam_' + c]['ExpSetting']
      this._shuttlingPose = info['Cam_' + c]['ShuttlePose']
      this._isPartMoving = info['Cam_' + c]['IsMoving']
    } else {
      this._slaveCmdInfos.featureMask = info['Cam_' + c]['FeatureMask']
      this._slaveCmdInfos.exposureSet = info['Cam_' + c]['ExpSetting']
      this._slaveCmdInfos.shuttlingPose = info['Cam_' + c]['ShuttlePose']
      this._slaveCmdInfos.isPartMoving = info['Cam_' + c]['IsMoving']
      if (c === this._partID1) {
        this._slaveCmdInfos.isCameraMoving = isCameraMoving
      } else {
        this._slaveCmdInfos.isCameraMoving = 0
      }
    }
  }

  this._sendToSlave = info['SendToSlave']
  return true
}
/**
 * Same as ...PartID
 * The informations are coming through the camera ID and 
 * the passed parameters
 * @param {any} featureMask
 * @param {any} expSetting
 * @param {any} shuttlingPose
 * @param {any} isPartMoving
 */
CommandBase.prototype.fillCmdInfoWithCameraID = function (featureMask, expSetting, shuttlingPose, isPartMoving) {
 
  if (!g_LooukupTables.cameraLookup.hasOwnProperty(this._index)) {
    this._validCMD = ECodes.E_INVALID_ARGUMENT
    return false
  }

  let index = parseInt(this._splittedCmd[1])
  let isCameraMoving = (index & MASK_MOVING_CAMERA) >> SHIFT_MOVING_CAMERA

  let info = g_LooukupTables.cameraLookup[this._index]

  for (let c = 1; c <= MAX_CAMERAS; c++) {
    if (c == this._myIndex) {
      this._featureMask = featureMask
      this._exposureSet = expSetting
      this._shuttlingPose = shuttlingPose
      this._isPartMoving = isPartMoving
    } else {
      this._slaveCmdInfos.featureMask = featureMask
      this._slaveCmdInfos.exposureSet = expSetting
      this._slaveCmdInfos.shuttlingPose = shuttlingPose
      this._slaveCmdInfos.isPartMoving = isPartMoving
      if (c === this._partID1) {
        this._slaveCmdInfos.isCameraMoving = isCameraMoving
      } else {
        this._slaveCmdInfos.isCameraMoving = 0
      }
    }
  }
  this._sendToSlave = info['SendToSlave']

  return true
}
/**
 * Collects all results from the result buffer
 * Is used in the XI command to create the result string
 * */
CommandBase.prototype.getAllResultData = function () {
  let resultStr = ''
  for (var c = 1; c <= MAX_CAMERAS; c++) {
    if (this._results[c].isNeeded > 0) {
      if (this._results[c].isValid > 0) {
        if (this._results[c].state > 0) {
          if (this._results[c].hasOwnProperty('data')) {
            let dat = this._results[c].data
            for (let a = 0; a < dat.length; a++) {
              for (let d in dat[a]) {
                resultStr = resultStr + ',' + dat[a][d]
              }
            }
          }
        }
      }
    }
  }
  return resultStr
}
/**
 * Goes through the result structure and returns the overall state of all single results
 * */
CommandBase.prototype.checkResultsState = function () {
  for (var c = 1; c <= MAX_CAMERAS; c++) {
    if (this._results[c].isNeeded > 0) {
      if (this._results[c].isValid > 0) {
        if (this._results[c].state <= 0) {
          this._results.error = this._results[c].state
        }
      }
    }
  }
}

/*
 * 
1010 GS,<cam>
1011 GV,<cam>

1050 TM,<cam>,<hw trigger>

2020 HEB,<cam>
2021 HE,<cam>,<featureID>,<X>,<Y>,<Z>,<A>,<B>,<C>
2022 HEE,<cam>

2030 ACB,<cam>,<featureID>,<X>,<Y>,<Z>,<A>,<B>,<C>
2031 AC,<cam>,<featureID>,<X>,<Y>,<Z>,<A>,<B>,<C>

2040 CCB,<cam>,<SwapHandedness>,<FeatureOffsetX>,<FeatureOffsetY>
2041 CC,<cam>,<featureID>,<X>,<Y>,<Z>,<A>,<B>,<C>
2042 CCE,<cam>

3010 TA,<partID>
3011 XA,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
3012 XAS,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
3013 XALC,<partID>,<resultMode>,<lcheckMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
3014 XAMP,<partIDs>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>

3020 SGP,<stepID>,<coords>,<X>,<Y>,<Theta>
3021 GGP,<stepID>,<coord>
3022 GCP,<stepID>,<coord>,<X>,<Y>,<Z>,<A>,<B>,<C>

4010 TT,<partID>,<X>,<Y>,<Z>,<A>,<B>,<C>
4011 TTR,<partID>,<X>,<Y>,<Z>,<A>,<B>,<C>
4012 XT,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
4013 XTS,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
4014 XTLC,<partID>,<resultMode>,<lcheckMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
4015 XTMP,<partIDs>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
4016 XT2,<partIDs>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>

5010 CP
5011 LF,<stepID>,<ProductID>,<X>,<Y>,<Z>,<A>,<B>,<C>
5012 TP,<partID>,<AlignMode>
5013 TPR,<partID>,<AlignMode>,<X>,<Y>,<Z>,<A>,<B>,<C>

5014 GP,<partID>,<AlignMode>,<ResultMode>, [current motion pose]

5015 LC,<partID>,<lcheckMode>

5050 PID,<ProductID>

*/

var masterCommands = {
  //  GS,<cam>
  //  GS,<Status>
  // SLMP: 1010
  GS: function (myIndex, cmdString) { // Camera ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 1010
    this._numArguments = 0
    this._hasIndex = 1
    let types = [ARG_TYPES.INT] // camera id

    // for compatibility with older version
    // if there is no camera id that means all cameras (was allowed in older versions)
    if (this._splittedCmd.length < 2) {
      this._splittedCmd.push('0')
    }
    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }

    if (this.setIndex() != true) {
      return
    }

    if ((isValidCameraID(this._index) != true) && (this._index != 0)) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }
    
    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }

    this._validCMD = 1
    this.execute = function (t) {
      tracer.addMessage('-> GS: Execute ' + timeTracker.getElapsedTime())
      let major = InSightFunctions.fnGetCellValue('Version.Major')
      let minor = InSightFunctions.fnGetCellValue('Version.Minor')
      let subMinor = InSightFunctions.fnGetCellValue('Version.SubMinor')
      let jobName = InSightFunctions.fnGetCellValue('Internal.Recipe.JobName')

      this._results[this._myIndex].data.push({ 'Major': major, 'Minor': minor, 'SubMinor': subMinor, 'JobName': jobName })
      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1
      tracer.addMessage('<- GS: Execute ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_SLAVE_RESULT
    }
    this.computeTotalResult = function () {
      tracer.addMessage('-> GS: Compute result ' + timeTracker.getElapsedTime())
      // TODO: //Prüfung ob alle Versionen gleich sind
      let plcString = ''
      let retState = 1
      let major = -1
      let minor = -1
      let subMinor = -1
      let jobName = ''

      this.checkResultsState()

      if (this._results.error == ECodes.E_NO_ERROR) {
        for (let c = 1; c <= MAX_CAMERAS; c++) {
          if (this._results[c].isNeeded > 0) {
            if (major == -1) {
              major = this._results[c].data[0].Major
              minor = this._results[c].data[0].Minor
              subMinor = this._results[c].data[0].SubMinor
            } else {
              let data = this._results[c].data[0]
              if ((major != data.Major) || (minor != data.Minor) || (subMinor != data.SubMinor)) {
                retState = ECodes.E_DIFFERENT_VERSIONS
              }
            }

            if (jobName.length == 0) {
              jobName = (this._results[c].data[0].JobName).toLowerCase()
            } else {
              if (jobName != (this._results[c].data[0].JobName).toLowerCase()) {
                retState = ECodes.E_DIFFERENT_JOB_NAMES
              }
            }
          }
        }
      } else {
        retState = this._results.error
      }

      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], retState)
      tracer.addMessage('-> GS: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  //  GV,<cam>
  //  GV,<Status>,<Major>,<Minor>,<SubMinor>
  // SLMP: 1011
  GV: function (myIndex, cmdString) { // Camera ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 1011
    this._numArguments = 0
    this._hasIndex = 1
    let types = [ARG_TYPES.INT] // camera id
    // for compatibility with older version
    // if there is no camera id that means all cameras (was allowed in older versions)
    if (this._splittedCmd.length < 2) {
      this._splittedCmd.push('0')
    }
    if (this.checkLength() != true) {
      return
    }
    if (this.checkArgTypes(types) != true) {
      return
    }
    if (this.setIndex() != true) {
      return
    }
    if ((isValidCameraID(this._index) != true) && (this._index !== 0)) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }
    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }

    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> GV: Execute')
      let major = InSightFunctions.fnGetCellValue('Version.Major')
      let minor = InSightFunctions.fnGetCellValue('Version.Minor')
      let subMinor = InSightFunctions.fnGetCellValue('Version.SubMinor')

      this._results[this._myIndex].data.push({ 'Major': major, 'Minor': minor, 'SubMinor': subMinor })
      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1
      tracer.addMessage('<- GV: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> GV: Compute result')
      // TODO: //Prüfung ob alle Versionen gleich sind
      let plcString = ''
      let resStr = ''
      let retState = 1
      let major = -1
      let minor = -1
      let subMinor = -1

      this.checkResultsState()

      if (this._results.error == ECodes.E_NO_ERROR) {
        for (let c = 1; c <= MAX_CAMERAS; c++) {
          if (this._results[c].isNeeded > 0) {
            if (major == -1) {
              major = this._results[c].data[0].Major
              minor = this._results[c].data[0].Minor
              subMinor = this._results[c].data[0].SubMinor
            } else {
              let data = this._results[c].data[0]
              if ((major != data.Major) || (minor != data.Minor) || (subMinor != data.SubMinor)) {
                retState = ECodes.E_DIFFERENT_VERSIONS
              }
            }
          }
        }
      } else {
        retState = this._results.error
      }

      if (retState > 0) {
        resStr = InSightFunctions.fnStringf(',%d,%d,%d', major, minor, subMinor)
      }
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)
      tracer.addMessage('-> GV: Compute result')
      return plcString
    }
  },

  // TM,<cam>,<hw trigger> (0=off/1=on)
  // TM,<Status>
  // SLMP: 1050
  TM: function (myIndex, cmdString) { // Camera ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 1050
    this._numArguments = 1
    this._hasIndex = 1
    let types = [ARG_TYPES.INT, // camera id
      ARG_TYPES.INT] // trigger mode

    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }
    if (this.setIndex() != true) {
      return
    }

    if (isValidCameraID(this._index) != true) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }

    var triggerMode = parseInt(this._splittedCmd[2])
    if (!(triggerMode == 1 || triggerMode == 0)) {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return
    }
    
    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }

    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> TM: Execute')
      if (triggerMode == 0) {
        triggerMode = 32
      } else {
        triggerMode = 0
      }
      t.triggerMode = triggerMode

      InSightFunctions.fnSetCellValue('TriggerMode', triggerMode)
      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1

      tracer.addMessage('<- TM: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  // HEB,<cam>
  // HEB,<Status>
  // SLMP: 2020
  HEB: function (myIndex, cmdString) { // Camera ID 0/1/2
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2020
    this._numArguments = 0
    this._hasIndex = 1
    let types = [ARG_TYPES.INT] // camera id
    
    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }
    if (this.setIndex() != true) {
      return
    }

    let index = this._index
    this._isCameraMoving = ((index & MASK_MOVING_CAMERA) >> SHIFT_MOVING_CAMERA) || g_Settings.IsRobotMounted

    // remove the upper bits
    this._index = this._index & MASK_CAM_ID

    if ((isValidCameraID(this._index) != true)&& (this._index != 0)){
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }     
    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> HEB: Execute')
      InSightFunctions.fnSetCellValue('HECalibration.IsCameraMoving', this._isCameraMoving)

      g_HECalibrationSettings = new HECalibrationSettings()

      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1

      tracer.addMessage('<- HEB: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  // HE,<cam>,<targetID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // HE,<Status>
  // SLMP: 2021
  HE: function (myIndex, cmdString) { // Camera ID 0/1/2
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2021
    this._numArguments = 7
    this._hasIndex = 1
    let types = [ARG_TYPES.INT, // camera id
      ARG_TYPES.INT, // target id
      ARG_TYPES.FLOAT, // x
      ARG_TYPES.FLOAT, // y
      ARG_TYPES.FLOAT, // z
      ARG_TYPES.FLOAT, // a
      ARG_TYPES.FLOAT, // b
      ARG_TYPES.FLOAT] // c

    if (this.checkLength() != true) {
      return
    }
    if (this.checkArgTypes(types) != true) {
      return
    }
    if (this.setIndex() != true) {
      return
    }
    if ((isValidCameraID(this._index) != true) && (this._index != 0)) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    } 
    
    var featureID = parseInt(this._splittedCmd[2])
    if (featureID == 0) {
      featureID = 1
    }
    
    if (featureID > MAX_FEATURES_PER_CAMERA) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }

    this.copyRobotPose(3)
    
    let fID = -1
    let expSettings = 1
    let shuttlingPose = 1

    if (this._index == 0) {
      expSettings = 1
      shuttlingPose = 1
    } else {
      fID = getFeatureIDFromCameraAndCameraFeatureID(this._index, featureID)    

      expSettings = getExpSettingsFromFeatureID(fID)
      shuttlingPose = getShuttlingPosFromFeatureID(fID)
    }

    if (fID == -1) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
    }
    
    if (this.fillCmdInfoWithCameraID(0xff & (1 << (featureID - 1)), expSettings, shuttlingPose, 0) !== true) {
      return
    }
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> Execute ' + timeTracker.getElapsedTime())

      if (this._useAsStepID > 0) {
        if (g_LooukupTables.stepLookup[this._index]['Cam_' + this._myIndex]['Enabled'] == 0) {
          tracer.addMessage('------------------------------------ Error ')
        }
      }

      if (this._useAsPartID > 0) {
        if (g_LooukupTables.partLookup[this._partID1]['Cam_' + this._myIndex]['Enabled'] == 0) {
          tracer.addMessage('------------------------------------ Error ')
        }
      }
      this._enabledFeatures = 0
            
      setFeatureAcqSettings(this._exposureSet - 1)

      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 1)
      }
      if (g_HECalibrationSettings != null) {
        if (g_HECalibrationSettings.calibrationID == -1) {
          g_HECalibrationSettings.calibrationID = this._shuttlingPose
          g_Calibrations[this._shuttlingPose].deleteCalibrationDataFile()
          t.onReloadHeCalibrations()
          g_Calibrations[this._shuttlingPose] = new Calibration(this._shuttlingPose)
          g_Calibrations[this._shuttlingPose]['calibrationData']['isMoving'] = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
         
          InSightFunctions.fnSetCellValue('HECalibration.' + this._shuttlingPose + '.IsCameraMoving', g_Calibrations[this._shuttlingPose]['calibrationData']['isMoving'])
        } else if (g_HECalibrationSettings.calibrationID != this._shuttlingPose) {
          
          this._results.error = ECodes.E_NOT_SUPPORTED
        }
      } else {
        this._results.error = ECodes.E_NO_START_COMMAND
      }

      if(this._results.error== ECodes.E_NO_ERROR){
        InSightFunctions.fnSetEvent(32)
      }

      tracer.addMessage('<- Execute ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_IMAGE_ACQUIRED
    }

    this.toolsDone = function (t) {
      tracer.addMessage('-> HE: Tools done')      
      g_Graphics['ShowCalibrationPoints_' + this._shuttlingPose] = 1            
      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')
      let targetTrained = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained')
      this._results[this._myIndex].isValid = 1

      if (targetTrained > 0) {
        if (targetValid > 0) {
          var targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          var targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          var targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

          g_Calibrations[this._shuttlingPose].calibrationData.targetX.push(targetX)
          g_Calibrations[this._shuttlingPose].calibrationData.targetY.push(targetY)
          g_Calibrations[this._shuttlingPose].calibrationData.targetTheta.push(targetAngle)
          g_Calibrations[this._shuttlingPose].calibrationData.targetValid.push(targetValid)
          g_Calibrations[this._shuttlingPose].calibrationData.robotX.push(this._robotPose.x)
          g_Calibrations[this._shuttlingPose].calibrationData.robotY.push(this._robotPose.y)
          g_Calibrations[this._shuttlingPose].calibrationData.robotTheta.push(this._robotPose.thetaZ)

          g_Calibrations[this._shuttlingPose].calibrationData.count = g_Calibrations[this._shuttlingPose].calibrationData.targetX.length
         
          this._results[this._myIndex].state = 1

        } else {
          this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_FOUND
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
      }      

      tracer.addMessage('<- HE: Tools done')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  // HEE,<cam>
  // HEE,<Status>
  // SLMP: 2022
  HEE: function (myIndex, cmdString) { // Camera ID 0/1/2
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2022
    this._numArguments = 0
    this._hasIndex = 1
    var selectedCalibration = -1
    let types = [ARG_TYPES.INT] // camera id

    if (this.checkLength() != true) {
      return
    }
    if (this.checkArgTypes(types) != true) {
      return
    }
    if (this.setIndex() != true) {
      return
    }
    if ((isValidCameraID(this._index) != true) && (this._index != 0)) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }  

    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }

    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> HEE: Execute')
      if (g_HECalibrationSettings == null) {
        this._results.error = ECodes.E_NO_START_COMMAND
        return
      }
      selectedCalibration = g_HECalibrationSettings.calibrationID
      if (selectedCalibration > 0) {
        g_Graphics['ShowCalibrationPoints_' + selectedCalibration] = 1
        this._results[this._myIndex].isValid = 1
        let isMoving = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
        var valid = t.myCalibrations.CheckData(g_Calibrations[selectedCalibration].calibrationData, isMoving)
        if (valid > 0) {
          g_Calibrations[selectedCalibration].computeCalibration()
          if (g_Calibrations[selectedCalibration].runstatus > 0) {
            g_Calibrations[selectedCalibration].saveCalibrationDataToFile()
            InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 1)
            this._results[this._myIndex].state = 1
          } else {
            this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
          }
        } else {
          this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
      }

      if (this._results[this._myIndex].state != 1) {
        this._results.error = this._results[this._myIndex].state
      }

      g_HECalibrationSettings = null
      
      tracer.addMessage('<- HEE: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  // ACB,<cam>,<targetID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // ACB,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 2030
  ACB: function (myIndex, cmdString) { // Camera ID 1/2
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2030
    this._numArguments = 7
    this._hasIndex = 1
    let types = [ARG_TYPES.INT, // camera id
    ARG_TYPES.INT, // target id
    ARG_TYPES.FLOAT, // x
    ARG_TYPES.FLOAT, // y
    ARG_TYPES.FLOAT, // z
    ARG_TYPES.FLOAT, // a
    ARG_TYPES.FLOAT, // b
    ARG_TYPES.FLOAT] // c
       

    if (this.checkLength() != true) {
      return
    }
    if (this.checkArgTypes(types) != true) {
      return
    }
    if (this.setIndex() != true) {
      return
    }
    var featureID = parseInt(this._splittedCmd[2])
    if (featureID == 0) {
      featureID = 1
    }    

    if (featureID > MAX_FEATURES_PER_CAMERA) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }

    let index = this._index
    this._isCameraMoving = ((index & MASK_MOVING_CAMERA) >> SHIFT_MOVING_CAMERA) || g_Settings.IsRobotMounted
    // remove the upper bits
    this._index = this._index & MASK_CAM_ID

    if (isValidCameraID(this._index) != true) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }
    
    this.copyRobotPose(3)
    
    let fID = getFeatureIDFromCameraAndCameraFeatureID(this._index, featureID)
    if (fID < 0) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }
    let expSettings = getExpSettingsFromFeatureID(fID)
    if (expSettings < 0) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }
    
    let shuttlingPose = getShuttlingPosFromFeatureID(fID)
    if (shuttlingPose < 0) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }
    
    if (this.fillCmdInfoWithCameraID(0xff & (1 << (featureID - 1)), expSettings, shuttlingPose, 0) !== true) {
      return
    }

    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> ACB: Tools done')

      InSightFunctions.fnSetCellValue('HECalibration.Selector', this._shuttlingPose-1)      
      InSightFunctions.fnSetCellValue('HECalibration.IsCameraMoving', this._isCameraMoving)
      InSightFunctions.fnSetCellValue('HECalibration.' + this._shuttlingPose + '.IsCameraMoving', this._isCameraMoving)

      g_AutoCalibRuntime = {}
      g_AutoCalibRuntime = g_Settings.AutoCalibration
      g_AutoCalibRuntime['innerDist'] = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Trainregion.InnerDist')
      g_AutoCalibRuntime['loopCnt'] = 0
      g_AutoCalibRuntime['stepCount'] = 0
      g_AutoCalibRuntime['direction'] = 1
      g_AutoCalibRuntime['minDist_Pixel'] = 55

      g_AutoCalibRuntime['rotAngle'] = (g_AutoCalibRuntime['AngleMax'] - g_AutoCalibRuntime['AngleMin']) / 10
      g_AutoCalibRuntime['advCalibPoints'] = []

      g_AutoCalibRuntime['lastMoveDistance_X'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM
      g_AutoCalibRuntime['lastMoveDistance_Y'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM
      g_AutoCalibRuntime['angleCompX'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM
      g_AutoCalibRuntime['angleCompY'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM

      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')

      if (InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained') > 0) {
        if (targetValid > 0) {
          let firstPos = { 'X': this._robotPose.x, 'Y': this._robotPose.y, 'Z': this._robotPose.z, 'A': this._robotPose.thetaZ, 'B': this._robotPose.thetaY, 'C': this._robotPose.thetaX }
          g_AutoCalibRuntime['FirstPos'] = firstPos
          g_AutoCalibRuntime['PreCalibration'] = { 'Calibrated': 0 }
          g_AutoCalibRuntime['PreCalibPoses'] = []
          g_AutoCalibRuntime['CalibPoints'] = []
          g_AutoCalibRuntime['RobCalibPoses'] = []
          g_AutoCalibRuntime['NextRobotPose'] = {}
          g_AutoCalibRuntime['Compensation'] = {}
          g_AutoCalibRuntime['LastNextPos'] = { 'X': 0, 'Y': 0, 'Z': 0, 'A': 0, 'B': 0, 'C': 0 }

          g_Graphics['ShowCalibrationPoints_' + this._shuttlingPose] = 1

          g_Calibrations[this._shuttlingPose].deleteCalibrationDataFile()
          t.onReloadHeCalibrations()
          
          t.myCalibrations.calibrations[this._shuttlingPose] = new Calibration(this._shuttlingPose)
          t.myCalibrations.calibrations[this._shuttlingPose]['calibrationData']['isMoving'] = this._isCameraMoving

          var targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          var targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          var targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

          let nextRobotPose = t.myCalibrations.doAutoCalibration(this._robotPose.x, this._robotPose.y, this._robotPose.z, this._robotPose.thetaZ, this._robotPose.thetaY, this._robotPose.thetaX, targetX, targetY, targetAngle, targetValid)

          if (nextRobotPose.Valid) {
            nextRobotPose.NextX = Math.round(nextRobotPose.NextX * 10000) / 10000
            nextRobotPose.NextY = Math.round(nextRobotPose.NextY * 10000) / 10000
            nextRobotPose.NextAngle = Math.round(nextRobotPose.NextAngle * 10000) / 10000
          }
          this._results[t.myDetails.myIndex].isValid = 1
          this._results[t.myDetails.myIndex].state = nextRobotPose.Valid
          this._results[t.myDetails.myIndex].data = nextRobotPose
        } else {
          this._results[t.myDetails.myIndex].isValid = 1
          this._results[t.myDetails.myIndex].state = ECodes.E_FEATURE_NOT_FOUND
        }
      } else {
        this._results[t.myDetails.myIndex].isValid = 1
        this._results[t.myDetails.myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
      }
      tracer.addMessage('<- ACB: Tools done')
      return States.WAITING_FOR_SLAVE_RESULT
    }
    this.computeTotalResult = function () {
      tracer.addMessage('-> ACB: Compute result')
      let plcString = ''
      this.checkResultsState()

      if (this._results.error == ECodes.E_NO_ERROR) {
        let res = this._results[this._index]
        plcString = InSightFunctions.fnStringf('%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', this._splittedCmd[0], res.state, res.data.NextX, res.data.NextY, this._robotPose.z, res.data.NextAngle, this._robotPose.thetaY, this._robotPose.thetaX)
      } else {
        plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], this._results.error)
      }
      tracer.addMessage('<- ACB: Compute result')
      return plcString
    }
  },

  // AC,<cam>,<targetID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // AC,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 2031
  AC: function (myIndex, cmdString) { // Camera ID 1/2
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2031
    this._numArguments = 7
    this._hasIndex = 1

    let types = [ARG_TYPES.INT, // camera id
    ARG_TYPES.INT, // target id
    ARG_TYPES.FLOAT, // x
    ARG_TYPES.FLOAT, // y
    ARG_TYPES.FLOAT, // z
    ARG_TYPES.FLOAT, // a
    ARG_TYPES.FLOAT, // b
    ARG_TYPES.FLOAT] // c

    if (this.checkLength() != true) {
      return
    }
    if (this.checkArgTypes(types) != true) {
      return
    }
    if (this.setIndex() != true) {
      return
    }
    var featureID = parseInt(this._splittedCmd[2])
    if (featureID == 0) {
      featureID = 1
    }

    if (featureID > MAX_FEATURES_PER_CAMERA) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }

    if (isValidCameraID(this._index) != true) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }
    if (featureID > MAX_FEATURES_PER_CAMERA) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }

    this.copyRobotPose(3)
    
    let fID = getFeatureIDFromCameraAndCameraFeatureID(this._index, featureID)
    if (fID < 0) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }
    let expSettings = getExpSettingsFromFeatureID(fID)
    if (expSettings < 0) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }

    let shuttlingPose = getShuttlingPosFromFeatureID(fID)
    if (shuttlingPose < 0) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }

    if (this.fillCmdInfoWithCameraID(0xff & (1 << (featureID - 1)), expSettings, shuttlingPose, 0) !== true) {
      return
    }
    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> AC Tools done')

      var nextRobotPose = { 'Valid': 0 }
      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')

      if (InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained') > 0) {
        if (targetValid > 0) {
          let targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          let targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          let targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

          if (g_AutoCalibRuntime['PreCalibration'].Calibrated == 0) {
            nextRobotPose = t.myCalibrations.doAutoCalibration(this._robotPose.x, this._robotPose.y, this._robotPose.z, this._robotPose.thetaZ, this._robotPose.thetaY, this._robotPose.thetaX, targetX, targetY, targetAngle, targetValid)
          } else {
          
            g_Graphics['ShowCalibrationPoints_' + this._shuttlingPose] = 1          
            let calib = t.myCalibrations.calibrations[this._shuttlingPose]

            calib.calibrationData.targetX.push(targetX)
            calib.calibrationData.targetY.push(targetY)
            calib.calibrationData.targetTheta.push(targetAngle)
            calib.calibrationData.targetValid.push(targetValid)
            calib.calibrationData.robotX.push(this._robotPose.x)
            calib.calibrationData.robotY.push(this._robotPose.y)
            calib.calibrationData.robotTheta.push(this._robotPose.thetaZ)

            calib.calibrationData.count = calib.calibrationData.targetX.length

            nextRobotPose = t.myCalibrations.doAutoCalibration(this._robotPose.x, this._robotPose.y, this._robotPose.z, this._robotPose.thetaZ, this._robotPose.thetaY, this._robotPose.thetaX, targetX, targetY, targetAngle, targetValid)

            if (nextRobotPose.Valid == 1) {
              let isMoving = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
              var valid = t.myCalibrations.CheckData(calib.calibrationData, isMoving)
              if (valid > 0) {
                calib.computeCalibration()
                if (calib.runstatus > 0) {
                  calib.saveCalibrationDataToFile()
                  InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 1)
                } else {

                }
              } else {
                console.log('Invalid calibration data! Calibration not saved!')                
              }
            }
          }

          if (nextRobotPose.Valid) {
            nextRobotPose.NextX = Math.round(nextRobotPose.NextX * 10000) / 10000
            nextRobotPose.NextY = Math.round(nextRobotPose.NextY * 10000) / 10000
            nextRobotPose.NextAngle = Math.round(nextRobotPose.NextAngle * 10000) / 10000
          }

          this._results[t.myDetails.myIndex].isValid = 1
          this._results[t.myDetails.myIndex].state = nextRobotPose.Valid
          this._results[t.myDetails.myIndex].data = nextRobotPose
        } else {
          this._results[t.myDetails.myIndex].isValid = 1
          this._results[t.myDetails.myIndex].state = ECodes.E_FEATURE_NOT_FOUND
          this._results[t.myDetails.myIndex].data = {}
        }
      } else {
        this._results[t.myDetails.myIndex].isValid = 1
        this._results[t.myDetails.myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
        this._results[t.myDetails.myIndex].data = {}
      }
      tracer.addMessage('<- AC Tools done')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> AC compute total result')
      let plcString = ''
      let errCnt = 0
      let errCode
      for (var i = 1; i <= MAX_CAMERAS; i++) {
        if (this._results[i].isNeeded > 0) {
          if (this._results[i].isValid > 0) {
            if (this._results[i].state <= 0) {
              errCnt++
              errCode = this._results[i].state
            }
          }
        }
      }
      let res = this._results[this._index]
      if (errCnt > 0) {
        plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], errCode)
      } else {
        plcString = InSightFunctions.fnStringf('%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', this._splittedCmd[0], res.state, res.data.NextX, res.data.NextY, this._robotPose.z, res.data.NextAngle, this._robotPose.thetaY, this._robotPose.thetaX)
      }
      tracer.addMessage('<- AC compute total result')
      return plcString
    }
  },

  // CCB,<cam>,<SwapHandedness>,<FeatureOffsetX>,<FeatureOffsetY> (0=don't swap / 1=swap)
  // CCB,<Status>
  // SLMP: 2040
  CCB: function (myIndex, cmdString) { // Camera ID 0/1/2
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2040
    this._numArguments = 3
    this._hasIndex = 1

    if (this.checkLengthAndSetIndex(0, MAX_CAMERAS) != true) {
      return
    }
    var swapHandedness = parseInt(this._splittedCmd[2])
    var featureOffsetX = parseFloat(this._splittedCmd[3])
    var featureOffsetY = parseFloat(this._splittedCmd[4])

    let index = parseInt(this._splittedCmd[1])
    this._isCameraMoving = ((index & MASK_MOVING_CAMERA) >> SHIFT_MOVING_CAMERA) || g_Settings.IsRobotMounted

    if (this._index > MAX_CAMERAS) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }

    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> CCB: Execute')
      InSightFunctions.fnSetCellValue('HECalibration.IsCameraMoving', this._isCameraMoving)
      InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 0)
      g_CustomCalibrationSettings = new CustomCalibrationSettings(swapHandedness, featureOffsetX, featureOffsetY, -1)

      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1

      tracer.addMessage('<- CCB: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  // CC,<cam>,<targetID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // CC,<Status>
  // SLMP: 2041
  CC: function (myIndex, cmdString) { // Camera ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2041
    this._numArguments = 7
    this._hasIndex = 1

    if (this.checkLengthAndSetIndex(0, MAX_CAMERAS) != true) {
      return
    }

    if (((this._index == this._myIndex) || (this._index == 0))  && (g_CustomCalibrationSettings == null)) {
      this._validCMD = ECodes.E_NO_START_COMMAND
      return
    }

    var featureID = parseInt(this._splittedCmd[2])
    if (featureID == 0) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }

    if (this._index > MAX_CAMERAS) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }
    if (featureID > MAX_FEATURES_PER_CAMERA) {
      this._validCMD = ECodes.E_INVALID_CAMERA_FEATURE_ID
      return
    }    

    let fID = -1
    let expSettings = 1
    let shuttlingPose = 1    

    if (this._index == 0) {      
      fID = getFeatureIDFromCameraAndCameraFeatureID(this._myIndex, featureID)
    } else {
      fID = getFeatureIDFromCameraAndCameraFeatureID(this._index, featureID)    
    }

    if (fID == -1) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }

    expSettings = getExpSettingsFromFeatureID(fID)
    shuttlingPose = getShuttlingPosFromFeatureID(fID) 
    if (this.fillCmdInfoWithCameraID(0xff & (1 << (featureID - 1)), expSettings, shuttlingPose, 0) !== true) {
      return
    }

    this._validCMD = 1  
    this.execute = function (t) {
        tracer.addMessage('-> Execute ' + timeTracker.getElapsedTime())

        this._enabledFeatures = 0

        setFeatureAcqSettings(this._exposureSet - 1)

        if (this._logImageType.length > 2) {
            InSightFunctions.fnSetCellValue(this._logImageType, 1)
        }

        if (t.triggerMode == 32) {
            InSightFunctions.fnSetEvent(32)
        }

        if ((this._index == this._myIndex) || (this._index == 0)) {
            this.copyRobotPose(3)

            if (g_CustomCalibrationSettings.calibrationID < 1) {
                g_CustomCalibrationSettings.calibrationID = shuttlingPose
                g_Calibrations[this._shuttlingPose].deleteCalibrationDataFile()
                t.onReloadHeCalibrations()
                g_Calibrations[shuttlingPose] = new Calibration(shuttlingPose)
                g_Calibrations[shuttlingPose]['calibrationData']['isMoving'] = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
                InSightFunctions.fnSetCellValue('HECalibration.' + shuttlingPose + '.IsCameraMoving', g_Calibrations[shuttlingPose]['calibrationData']['isMoving'])
            }
        }
        
        tracer.addMessage('<- CC init ' + timeTracker.getElapsedTime())
        tracer.addMessage('<- Execute ' + timeTracker.getElapsedTime())
        return States.WAITING_FOR_IMAGE_ACQUIRED
    }
    this.toolsDone = function (t) {
      tracer.addMessage('-> CC: Tools done')
      g_Graphics['ShowCalibrationPoints_' + this._shuttlingPose] = 1
      
      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')
      let targetTrained = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained')

      this._results[this._myIndex].isValid = 1
      if (targetTrained > 0) {
        if (targetValid > 0) {
          var targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          var targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          var targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

         g_Calibrations[this._shuttlingPose].calibrationData.targetX.push(targetX)
         g_Calibrations[this._shuttlingPose].calibrationData.targetY.push(targetY)
         g_Calibrations[this._shuttlingPose].calibrationData.targetTheta.push(targetAngle)
         g_Calibrations[this._shuttlingPose].calibrationData.targetValid.push(targetValid)
         g_Calibrations[this._shuttlingPose].calibrationData.robotX.push(this._robotPose.x)
         g_Calibrations[this._shuttlingPose].calibrationData.robotY.push(this._robotPose.y)
         g_Calibrations[this._shuttlingPose].calibrationData.robotTheta.push(this._robotPose.thetaZ)

         g_Calibrations[this._shuttlingPose].calibrationData.count = g_Calibrations[this._shuttlingPose].calibrationData.targetX.length

          this._results[this._myIndex].state = 1

        } else {
          this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_FOUND
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
      }

      tracer.addMessage('<- CC: Tools done')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  // CCE,<cam>
  // CCE,<Status>
  // SLMP: 2042
  CCE: function (myIndex, cmdString) { // Camera ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 2042
    this._numArguments = 0
    this._hasIndex = 1
    var selectedCalibration = 0

    if (this.checkLengthAndSetIndex(0, MAX_CAMERAS) != true) {
      return
    }

    if (((this._index == this._myIndex) || (this._index == 0)) && (g_CustomCalibrationSettings == null)) {
      this._validCMD = ECodes.E_NO_START_COMMAND
      return
    }
    if (this._index > MAX_CAMERAS) {
      this._validCMD = ECodes.E_INVALID_CAMERA_ID
      return
    }
        
    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }

    if ((this._index == this._myIndex) || (this._index == 0)) {
      selectedCalibration = g_CustomCalibrationSettings.calibrationID
    }

    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> CCE: Execute')
      
      if (selectedCalibration > 0) {
        g_Graphics['ShowCalibrationPoints_' + selectedCalibration] = 1
        this._results[this._myIndex].isValid = 1
        let isMoving = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
        var ccData = t.myCalibrations.doCustomCalibration(g_Calibrations[selectedCalibration].calibrationData,
          g_CustomCalibrationSettings.swapHandedness, g_CustomCalibrationSettings.featureOffsetX, g_CustomCalibrationSettings.featureOffsetY)

        if (ccData != 0) {
          var valid = t.myCalibrations.CheckData(g_Calibrations[selectedCalibration].calibrationData, isMoving)
          if (valid > 0) {
            g_Calibrations[selectedCalibration].computeCalibration()
            if (g_Calibrations[selectedCalibration].runstatus > 0) {
              g_Calibrations[selectedCalibration].saveCalibrationDataToFile()
              InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 1)
              this._results[this._myIndex].state = 1
            } else {
              this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
            }
          } else {
            this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
          }
        } else {
          this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
      }
      if (this._results[this._myIndex].state != 1) {
        this._results.error = this._results[this._myIndex].state
      }

      g_CustomCalibrationSettings = null      
      
      tracer.addMessage('<- CCE: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  // TA,<partID>
  // TA,<Status>
  // SLMP: 3010
  TA: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3010
    this._numArguments = 0
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsTrainImage'

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }

    if (this._splittedCmd.length >= 8) {
      if (this.checkArgumentsNumbers(2, this._splittedCmd.length - 1) == false) {
        return
      }
      this.copyRobotPose(2)
    }

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }

    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }
    this._validCMD = 1

    this.computeTotalResult = function () {
      tracer.addMessage('-> TA Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      let errCnt = 0
      let errCode = ECodes.E_NO_ERROR

      for (var i = 1; i <= MAX_CAMERAS; i++) {
        if (this._results[i].isNeeded > 0) {
          if (this._results[i].isValid > 0) {
            if (this._results[i].state <= 0) {
              errCnt++
              errCode = this._results[i].state
            }
          }
        }
      }

      let state = ECodes.E_INTERNAL_ERROR
      if (errCnt == 0) {
        state = 1 // No error
        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees

              g_TrainedFeatures[targetID][this._gripperID1].valid = g_RuntimeFeatures[targetID].valid
              g_TrainedFeatures[targetID][this._gripperID1].x = g_RuntimeFeatures[targetID].x
              g_TrainedFeatures[targetID][this._gripperID1].y = g_RuntimeFeatures[targetID].y
              g_TrainedFeatures[targetID][this._gripperID1].thetaInDegrees = g_RuntimeFeatures[targetID].thetaInDegrees

              resIndex++
            }
          }
        }
        InSightFunctions.fnSetEvent(83)
      } else {
        state = errCode
      }

      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], state)

      tracer.addMessage('<- TA Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XA,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C> (1=ABS, 2=OFF)
  // XA,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 3011
  XA: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3011
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    var resMode = ResultMode.ABS

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 3; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    resMode = this._splittedCmd[2]
    if (!((resMode == ResultMode.ABS) || (resMode == ResultMode.OFF) ||
       (resMode == ResultMode[ResultMode.ABS]) || (resMode == ResultMode[ResultMode.OFF]))) {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return
    }
    this.copyRobotPose(3)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }
    
    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> XA: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push(transformed)
        }
      }

      g_Graphics.ShowCrossHair = []
      let camInfo = g_LooukupTables.partLookup[this._partID1]['Cam_' + this._myIndex]

      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let targetID = camInfo['Feature_' + f]
        if (targetID > 0) {
          let imgPos = []
          let trained = g_TrainedFeatures[targetID][this._gripperID1].valid
          if (trained > 0) {
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[targetID][this._gripperID1].x, g_TrainedFeatures[targetID][this._gripperID1].y, g_TrainedFeatures[targetID][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }
      tracer.addMessage('<- XA: Tools done ' + timeTracker.getElapsedTime())      

      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XA: Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)

      this.checkResultsState()
      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }
          }
        }
        newRobPose = ComputeAlignMode_1(this._index, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid
      } else {
        retState = this._results.error
      }

      let resStr = ''
      if (retState > 0) {
        resStr = InSightFunctions.fnStringf(',%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)
      }

      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)

      tracer.addMessage('<- XA: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XAS,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // XAS,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>,<Score>
  // SLMP: 3012
  XAS: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3012
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    var resMode = ResultMode.ABS

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 3; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    resMode = this._splittedCmd[2]
    if (!((resMode == ResultMode.ABS) || (resMode == ResultMode.OFF) || (resMode == ResultMode[ResultMode.ABS]) || (resMode == ResultMode[ResultMode.OFF]))) {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return
    }
    this.copyRobotPose(3)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }
    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> XAS: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        let score = -1

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
            score = InSightFunctions.fnGetCellValue('Target.' + f + '.Pattern_Score')
            score = (Math.round(score * 1000) / 1000)
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push([transformed, score])
        }
      }

      g_Graphics.ShowCrossHair = []

      let camInfo = g_LooukupTables.partLookup[this._partID1]['Cam_' + this._myIndex]

      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let targetID = camInfo['Feature_' + f]
        if (targetID > 0) {
          let imgPos = []
          let trained = g_TrainedFeatures[targetID][this._gripperID1].valid
          if (trained > 0) {
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[targetID][this._gripperID1].x, g_TrainedFeatures[targetID][this._gripperID1].y, g_TrainedFeatures[targetID][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }

      tracer.addMessage('<- XA: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XAS: Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      let scores = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)

      this.checkResultsState()
      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex][0].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex][0].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex][0].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex][0].thetaInDegrees
              scores = scores + InSightFunctions.fnStringf(',%.3f', this._results[c].data[resIndex][1])
              resIndex++
            }
          }
        }
        newRobPose = ComputeAlignMode_1(this._index, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid
      } else {
        retState = this._results.error
      }

      let resStr = ''
      if (retState > 0) {
        resStr = InSightFunctions.fnStringf(',%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)
        resStr = resStr + scores
      }

      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)

      tracer.addMessage('<- XAS: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XALC,<partID>,<resultMode>,<lcheckMode>,<X>,<Y>,<Z>,<A>,<B>,<C> (1=ABS, 2=OFF) (1=ABS, 2=DIFF, 3=REL)
  // XALC,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 3013
  XALC: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3013
    this._numArguments = 8
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    let types = [ARG_TYPES.INT, //partIDs
      [ARG_TYPES.DEF_RESULT_MODE, ['ABS', 'OFF']], // resultMode
      [ARG_TYPES.DEF_LCHECK_RESULT_MODE, ['ABS', 'DIFF', 'REL']], // LCResultMode
      ARG_TYPES.FLOAT, // X
      ARG_TYPES.FLOAT, // Y
      ARG_TYPES.FLOAT, // Z
      ARG_TYPES.FLOAT, // A
      ARG_TYPES.FLOAT, // B
      ARG_TYPES.FLOAT] // C

    var resMode = ResultMode.ABS
    var lcResMode = LCheckResultMode.ABS

    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }

    if (this.setIndex() != true) {
      return
    }   

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 4; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    resMode = this._splittedCmd[2]
    lcResMode = this._splittedCmd[3]
    lcResMode = getLCheckResultModeAsStr(lcResMode)

    this.copyRobotPose(4)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }

    if (isValidPartID(this._partID1) != true) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }

    if (g_LooukupTables.partLookup[this._partID1].FeatureIDs.length != 2) {
      this._validCMD = ECodes.E_COMBINATION_NOT_ALLOWED
      return
    }

    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> XALC: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push(transformed)
        }
      }

      g_Graphics.ShowCrossHair = []
      let camInfo = g_LooukupTables.partLookup[this._partID1]['Cam_' + this._myIndex]

      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let targetID = camInfo['Feature_' + f]
        if (targetID > 0) {
          let imgPos = []
          let trained = g_TrainedFeatures[targetID][this._gripperID1].valid
          if (trained > 0) {
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[targetID][this._gripperID1].x, g_TrainedFeatures[targetID][this._gripperID1].y, g_TrainedFeatures[targetID][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }
      tracer.addMessage('<- XALC: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XALC: Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)

      this.checkResultsState()
      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }
          }
        }
        newRobPose = ComputeAlignMode_1(this._index, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid
      } else {
        retState = this._results.error
      }
      let retDist = 0
      let dists
      if (retState > 0) {
        dists = getPartsFeatureDistance(this._partID1, this._gripperID1)        
        if (dists['valid'] === true) {
          retDist = dists[lcResMode]
        } else {
          retState = dists['valid']
        }        
      }
      let resStr = ''
      if (retState > 0) {
        let jdg = 0
        if (dists['judge'] === true) jdg = 1;
        resStr = InSightFunctions.fnStringf(',%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d', newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX, retDist, jdg)
      }
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)
      tracer.addMessage('<- XALC: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XAMP,<partIDs>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C> (1=ABS, 2=OFF)
  // XAMP,<Status>,<partID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 3014
  XAMP: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3014
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    let types = [ARG_TYPES.INT, //partIDs
    [ARG_TYPES.DEF_RESULT_MODE, ['ABS', 'OFF']], // resultMode
    ARG_TYPES.FLOAT, // X
    ARG_TYPES.FLOAT, // Y
    ARG_TYPES.FLOAT, // Z
    ARG_TYPES.FLOAT, // A
    ARG_TYPES.FLOAT, // B
    ARG_TYPES.FLOAT] // C

    var resMode = ResultMode.ABS

    if (this.checkLength() != true) {
      return
    }    

    if (this.checkArgTypes(types) != true) {
      return
    }

    if (this.setIndex() != true) {
      return
    }    

    this.decodeIDsfromIndex()
    this.getNextID()

    this._partID1 = this._decodedIDs[this._decodedIDs[0]]
    resMode = this._splittedCmd[2]    

    this.copyRobotPose(3)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }

    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }
    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> XAMP: Tools done ' + timeTracker.getElapsedTime())

      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push(transformed)
        }
      }

      g_Graphics.ShowCrossHair = []
      let camInfo = g_LooukupTables.partLookup[this._partID1]['Cam_' + this._myIndex]

      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let targetID = camInfo['Feature_' + f]
        if (targetID > 0) {
          let imgPos = []
          let trained = g_TrainedFeatures[targetID][this._gripperID1].valid
          if (trained > 0) {
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[targetID][this._gripperID1].x, g_TrainedFeatures[targetID][this._gripperID1].y, g_TrainedFeatures[targetID][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }
      tracer.addMessage('<- XAMP: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XAMP: Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)

      this.checkResultsState()
      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }
          }
        }
        newRobPose = ComputeAlignMode_1(this._partID1, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid
      } else {
        retState = this._results.error
      }

      let resStr = ''
      if (retState > 0) {
        resStr = InSightFunctions.fnStringf(',%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', this._decodedIDs[this._decodedIDs[0]], newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)
      }
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)
      
     
      if (retState <= 0) {
          if (this.getNextID() == true) { 
          g_Graphics.ShowCrossHair = []
          plcString = 'REDO_FROM_EXECUTE'
          this._partID1 = this._decodedIDs[this._decodedIDs[0]]
          this.resetResultStructure()
          this.fillCmdInfoWithPartID()
        }
      }
      tracer.addMessage('<- XAMP: Compute result ' + timeTracker.getElapsedTime())
      return plcString //plcString
    }
  },

  // XI,<cam>,<inspection>
  // XI,<Status>
  // SLMP: 6010
  XI: function (myIndex, cmdString) {
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 6010
    this._numArguments = 0
    this._hasIndex = 1
    this._useAsPartID = 0
    this._logImageType = 'LogImage.IsProductionImage'
    this._sendToSlave = 0

    this._index = parseInt(this._splittedCmd[1])
    var inspectionID = parseInt(this._splittedCmd[2])

    if (this.fillCmdInfoWithCameraID(inspectionID, 0, 0, 0) !== true) {
      return
    }
    this._isCameraMoving = 0
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> XI: Execute')
      this._enabledFeatures = 0
      if (g_Inspections.hasOwnProperty(inspectionID)) {

        setInspectionAcqSettings(inspectionID, 1)

        if (this._logImageType.length > 2) {

          InSightFunctions.fnSetCellValue(this._logImageType, 1)

        }
        if (t.triggerMode == 32) {
          InSightFunctions.fnSetEvent(32)
        }
      } else {
        this._results.error = ECodes.E_INVALID_ARGUMENT
      }

      tracer.addMessage('<- XI: Execute ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_IMAGE_ACQUIRED
    }

    this.imgAcquired = function () {
      tracer.addMessage('-> XI: Image acquired' + timeTracker.getElapsedTime())
      this._featureMask = inspectionID << 8
      this._enabledFeatures = this._featureMask
      tracer.addMessage('<- XI: Image acquired' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_TOOLS_DONE
    }

    this.toolsDone = function (t) {
      tracer.addMessage('-> XI: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      tracer.addMessage('<- XI: Tools done ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XI: Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      let plcString_1 = ''
      let resultStr = ''

      tracer.addMessage(this._results)

      this.checkResultsState()

      let retState = ECodes.E_UNSPECIFIED
      if (this._results.error == ECodes.E_NO_ERROR) {
        retState = 1
        resultStr = this.getAllResultData()
      } else {
        retState = this._results.error
      }
      plcString_1 = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], retState)
      if ((resultStr.length + plcString_1.length) > 255) {
        retState = ECodes.E_RESULSTRING_TO_LONG
        resultStr = ''
      }

      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resultStr)

      tracer.addMessage('<- XI: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // SGP,<stepID>,<coords>,<X>,<Y>,<Theta> (1=HOME2D, 2=CAM2D, 3=RAW2D)
  // SGP,<Status>
  // SLMP: 3020
  SGP: function (myIndex, cmdString) { // Step ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3020
    this._numArguments = 4
    this._hasIndex = 1
    this._useAsStepID = 1
    
    var mode = -1
    var newGoldenPose = new Feature(0, 0, 0, 0)
    var featureID = -1

    if (this.checkLengthAndSetIndex(1, RECIPE_MAX_STEPS) != true) {
      return
    }

    if (!g_Steps.hasOwnProperty(this._index)) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 3; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    if (g_Steps[this._index].FeatureIDs.length > 1) {
      this._validCMD = ECodes.E_COMBINATION_NOT_ALLOWED
      return
    } else {
      featureID = g_Steps[this._index].FeatureIDs[0]
    }

    if ((this._splittedCmd[2] == CoordinateSystem.HOME2D) || (this._splittedCmd[2] == CoordinateSystem.CAM2D) || (this._splittedCmd[2] == CoordinateSystem.RAW2D) ||
      (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.HOME2D]) || (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.CAM2D]) || (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.RAW2D])) {
      mode = this._splittedCmd[2]
      newGoldenPose.x = parseFloat(this._splittedCmd[3])
      newGoldenPose.y = parseFloat(this._splittedCmd[4])
      newGoldenPose.thetaInDegrees = parseFloat(this._splittedCmd[5])
      newGoldenPose.valid = 1
    } else {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return
    }

    if (this.fillCmdInfoWithStepID() !== true) {
      return
    }

    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }    
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> SGP: Execute')

      this._results[this._myIndex].state = ECodes.E_UNSPECIFIED
      this._results[this._myIndex].isValid = 1

      if (false) {
        this._results[this._myIndex].state = ECodes.E_COMBINATION_NOT_ALLOWED
      } else {
        if ((mode == CoordinateSystem.HOME2D) || (mode == CoordinateSystem[CoordinateSystem.HOME2D])) {
          this._results[this._myIndex].state = 1
        } else
          if ((mode == CoordinateSystem.CAM2D) || (mode == CoordinateSystem[CoordinateSystem.CAM2D])) {
            let newGPInWorld = getWorldFromCam(g_Calibrations, this._shuttlingPose, newGoldenPose.x, newGoldenPose.y, newGoldenPose.thetaInDegrees, 0, 0, 0)
            newGoldenPose.valid = newGPInWorld.valid
            if (newGoldenPose.valid > 0) {
              newGoldenPose.x = newGPInWorld.x
              newGoldenPose.y = newGPInWorld.y
              newGoldenPose.thetaInDegrees = newGPInWorld.thetaInDegrees
            }
            this._results[this._myIndex].state = newGoldenPose.valid
          } else
            if ((mode == CoordinateSystem.RAW2D) || (mode == CoordinateSystem[CoordinateSystem.RAW2D])) {
              let newGPInWorld = getWorldFromImage(g_Calibrations, this._shuttlingPose, newGoldenPose.x, newGoldenPose.y, newGoldenPose.thetaInDegrees, 0, 0, 0)
              newGoldenPose.valid = newGPInWorld.valid
              if (newGoldenPose.valid > 0) {
                newGoldenPose.x = newGPInWorld.x
                newGoldenPose.y = newGPInWorld.y
                newGoldenPose.thetaInDegrees = newGPInWorld.thetaInDegrees
              }
              this._results[this._myIndex].state = newGoldenPose.valid
            }
      }

      this._results[this._myIndex].data.push(newGoldenPose)
      tracer.addMessage('<- SGP: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> SGP: Compute result')
      let plcString = ''
      this.checkResultsState()
      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        let stepInfo = g_Steps[this._index]
        if (stepInfo['FeatureIDs'].length == 1) {

          let featureID = stepInfo['FeatureIDs'][0]
          let c = g_LooukupTables['features'][featureID]['CameraID']

          g_RuntimeFeatures[featureID].valid = this._results[c].data[0].valid
          g_RuntimeFeatures[featureID].x = this._results[c].data[0].x
          g_RuntimeFeatures[featureID].y = this._results[c].data[0].y
          g_RuntimeFeatures[featureID].thetaInDegrees = this._results[c].data[0].thetaInDegrees

          g_TrainedFeatures[featureID][this._gripperID1].valid = g_RuntimeFeatures[featureID].valid
          g_TrainedFeatures[featureID][this._gripperID1].x = g_RuntimeFeatures[featureID].x
          g_TrainedFeatures[featureID][this._gripperID1].y = g_RuntimeFeatures[featureID].y
          g_TrainedFeatures[featureID][this._gripperID1].thetaInDegrees = g_RuntimeFeatures[featureID].thetaInDegrees

          InSightFunctions.fnSetEvent(83)
          retState = 1
        } else {
          retState = ECodes.E_UNSPECIFIED
        }
      } else {
        retState = this._results.error
      }

      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], retState)
      tracer.addMessage('<- SGP: Compute result')
      return plcString
    }
  },

  // GGP,<stepID>,<coord>
  // GGP,<Status>,<X>,<Y>,<Theta>
  // SLMP: 3021
  GGP: function (myIndex, cmdString) { // Step ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3021
    this._numArguments = 1
    this._hasIndex = 1
    this._useAsStepID = 1
    this._onlyForMaster = 0
    var mode = -1

    var featureID = -1

    if (this.checkLengthAndSetIndex(1, RECIPE_MAX_STEPS) != true) {
      return
    }
    if (!g_Steps.hasOwnProperty(this._index)) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 3; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    if (g_Steps[this._index].FeatureIDs.length > 1) {
      this._validCMD = ECodes.E_COMBINATION_NOT_ALLOWED
      return
    } else {
      featureID = g_Steps[this._index].FeatureIDs[0]
    }

    if ((this._splittedCmd[2] == CoordinateSystem.HOME2D) || (this._splittedCmd[2] == CoordinateSystem.CAM2D) || (this._splittedCmd[2] == CoordinateSystem.RAW2D) ||
      (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.HOME2D]) || (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.CAM2D]) || (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.RAW2D])) {
      mode = this._splittedCmd[2]
    } else {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return
    }

    if (this.fillCmdInfoWithStepID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> GGP: Execute')

      this._results[this._myIndex].state = ECodes.E_UNSPECIFIED
      this._results[this._myIndex].isValid = 1

      if (false) {
        this._results[this._myIndex].state = ECodes.E_COMBINATION_NOT_ALLOWED
      } else {
        if (g_TrainedFeatures[featureID][this._gripperID1].valid > 0) {
          let goldenPose = cloneObj(g_TrainedFeatures[featureID][this._gripperID1])
          if ((mode == CoordinateSystem.HOME2D) || (mode == CoordinateSystem[CoordinateSystem.HOME2D])) {

          } else if ((mode == CoordinateSystem.CAM2D) || (mode == CoordinateSystem[CoordinateSystem.CAM2D])) {
            let goldenPoseInCam2D = getCamFromWorld(g_Calibrations, this._shuttlingPose, goldenPose.x, goldenPose.y, goldenPose.thetaInDegrees)
            goldenPose.valid = goldenPoseInCam2D.valid
            if (goldenPose.valid > 0) {
              goldenPose.x = goldenPoseInCam2D.x
              goldenPose.y = goldenPoseInCam2D.y
              goldenPose.thetaInDegrees = goldenPoseInCam2D.thetaInDegrees
            }
          } else if ((mode == CoordinateSystem.RAW2D) || (mode == CoordinateSystem[CoordinateSystem.RAW2D])) {
            let goldenPoseInRaw2D = getImageFromWorld(g_Calibrations, this._shuttlingPose, goldenPose.x, goldenPose.y, goldenPose.thetaInDegrees, 0, 0, 0)
            goldenPose.valid = goldenPoseInRaw2D.valid
            if (goldenPose.valid > 0) {
              goldenPose.x = goldenPoseInRaw2D.x
              goldenPose.y = goldenPoseInRaw2D.y
              goldenPose.thetaInDegrees = goldenPoseInRaw2D.thetaInDegrees
            }
          }
          this._results[this._myIndex].state = goldenPose.valid
          this._results[this._myIndex].data.push(goldenPose)
        } else {
          this._results[this._myIndex].state = ECodes.E_TARGET_POSE_NOT_TRAINED
        }

        tracer.addMessage('<- GGP: Execute')
        return States.WAITING_FOR_SLAVE_RESULT
      }
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> GGP: Compute result')
      let plcString = ''
      let resultString = ''
      this.checkResultsState()

      let retState = this._results.error
      if (retState === ECodes.E_NO_ERROR) {
        for (let r in this._results) {
          if ((typeof this._results[r] === 'object') && (this._results[r].isNeeded == true)) {
            let data = this._results[r].data[0]
            resultString += InSightFunctions.fnStringf(',%.3f,%.3f,%.3f', data.x, data.y, data.thetaInDegrees)
            retState = this._results[r].isValid
          }
        }
      }
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resultString)

      tracer.addMessage('<- GGP: Compute result')
      return plcString
    }
  },

  // GCP,<stepID>,<coord>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // GCP,<Status>,<X>,<Y>,<Theta>
  // SLMP: 3022

  GCP: function (myIndex, cmdString) { // Step ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 3022
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsStepID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    var mode = -1

    if (this.checkLengthAndSetIndex(1, RECIPE_MAX_STEPS) != true) {
      return
    }

    if (!g_Steps.hasOwnProperty(this._index)) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 3; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    if (g_Steps[this._index].FeatureIDs.length > 1) {
      this._validCMD = ECodes.E_COMBINATION_NOT_ALLOWED
      return
    }

    if ((this._splittedCmd[2] == CoordinateSystem.HOME2D) || (this._splittedCmd[2] == CoordinateSystem.CAM2D) || (this._splittedCmd[2] == CoordinateSystem.RAW2D) ||
      (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.HOME2D]) || (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.CAM2D]) || (this._splittedCmd[2] == CoordinateSystem[CoordinateSystem.RAW2D])) {
      mode = this._splittedCmd[2]
    } else {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return
    }
    this.copyRobotPose(3)

    if (this.fillCmdInfoWithStepID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> GCP: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let home2D = new Feature(0, 0, 0, 0)
        let transformed = new Feature(0, 0, 0, 0)

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {

            if (this._results[this._myIndex].state >= 0) {
              home2D = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
              if ((mode == CoordinateSystem.HOME2D) || (mode == CoordinateSystem[CoordinateSystem.HOME2D])) {
                transformed.valid = home2D.valid
                if (transformed.valid > 0) {
                  transformed.x = home2D.x
                  transformed.y = home2D.y
                  transformed.thetaInDegrees = home2D.thetaInDegrees
                }

              } else if ((mode == CoordinateSystem.CAM2D) || (mode == CoordinateSystem[CoordinateSystem.CAM2D])) {
                transformed = getCamFromWorld(g_Calibrations, this._shuttlingPose, home2D.x, home2D.y, home2D.thetaInDegrees)
              } else if ((mode == CoordinateSystem.RAW2D) || (mode == CoordinateSystem[CoordinateSystem.RAW2D])) {
                transformed = getImageFromWorld(g_Calibrations, this._shuttlingPose, home2D.x, home2D.y, home2D.thetaInDegrees, 0, 0, 0)
              }

              this._results[this._myIndex].state = transformed.valid
            }

          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }

          this._results[this._myIndex].data.push(transformed)
        }
      }

      tracer.addMessage('<- GCP: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> GCP: Compute result')
      let plcString = ''
      let resultString = ''
      this.checkResultsState()

      let retState = this._results.error
      if (retState === ECodes.E_NO_ERROR) {
        for (let r in this._results) {
          if ((typeof this._results[r] === 'object') && (this._results[r].isNeeded == true)) {
            let data = this._results[r].data[0]
            resultString += InSightFunctions.fnStringf(',%.3f,%.3f,%.3f', data.x, data.y, data.thetaInDegrees)
            retState = this._results[r].isValid
          }
        }
      }
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resultString)

      tracer.addMessage('<- GCP: Compute result')
      return plcString
    }

  },

  // TT,<partID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // TT,<Status>
  // SLMP: 4010
  TT: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 4010
    this._numArguments = 6
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsTrainImage'

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }
    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 1; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    this.copyRobotPose(2)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    this._validCMD = 1

    this.computeTotalResult = function () {
      tracer.addMessage('-> TT Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      this.checkResultsState()
      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees

              g_TrainedFeatures[targetID][this._gripperID1].valid = g_RuntimeFeatures[targetID].valid
              g_TrainedFeatures[targetID][this._gripperID1].x = g_RuntimeFeatures[targetID].x
              g_TrainedFeatures[targetID][this._gripperID1].y = g_RuntimeFeatures[targetID].y
              g_TrainedFeatures[targetID][this._gripperID1].thetaInDegrees = g_RuntimeFeatures[targetID].thetaInDegrees

              resIndex++
            }
          }
        }
        retState = 1
        InSightFunctions.fnSetEvent(83)
      } else {
        retState = this._results.error
      }

      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], retState)
      tracer.addMessage('<- TT Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // TTR,<partID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // TTR,<Status>
  // SLMP: 4011
  TTR: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 4011
    this._numArguments = 6
    this._hasIndex = 1
    this._useAsPartID = 1
    this._onlyForMaster = 1

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }
    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }
    let len = this._numArguments + this._hasIndex + 1
    for (let i = 1; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    this._robotPose.x = parseFloat(this._splittedCmd[2])
    this._robotPose.y = parseFloat(this._splittedCmd[3])
    this._robotPose.z = parseFloat(this._splittedCmd[4])
    this._robotPose.thetaZ = parseFloat(this._splittedCmd[5])
    this._robotPose.thetaY = parseFloat(this._splittedCmd[6])
    this._robotPose.thetaX = parseFloat(this._splittedCmd[7])
    this._robotPose.valid = 1

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> TTR: Execute')

      g_Parts[this._index].trainedRobotPose[this._gripperID1].x = this._robotPose.x
      g_Parts[this._index].trainedRobotPose[this._gripperID1].y = this._robotPose.y
      g_Parts[this._index].trainedRobotPose[this._gripperID1].z = this._robotPose.z
      g_Parts[this._index].trainedRobotPose[this._gripperID1].thetaZ = this._robotPose.thetaZ
      g_Parts[this._index].trainedRobotPose[this._gripperID1].thetaY = this._robotPose.thetaY
      g_Parts[this._index].trainedRobotPose[this._gripperID1].thetaX = this._robotPose.thetaX
      g_Parts[this._index].trainedRobotPose[this._gripperID1].valid = 1

      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1

      tracer.addMessage('<- TTR: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> TTR: Compute result')
      let plcString = ''
      let state = 1
      if (this._results[this._myIndex].isValid > 0) {
        state = this._results[this._myIndex].state
        InSightFunctions.fnSetEvent(83)
      }
      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], state)
      tracer.addMessage('<- TTR: Compute result')
      return plcString
    }
  },

  // XT,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // XT,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 4012
  // Compute two parts are removed now we have XT2 command
  XT: function (myIndex, cmdString) { // Step ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 4012
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    var resMode = ResultMode.ABS

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }
    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }
    
    let len = this._numArguments + this._hasIndex + 1
    for (let i = 3; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }
    
    resMode = this._splittedCmd[2]
    
    if (isValidResultMode(resMode) != true) {
      this._validCMD = ECodes.E_INVALID_RESULT_MODE
      return
    }    

    if ((resMode == ResultMode.GC) || (resMode == ResultMode[ResultMode.GC])) {
      asLogger.addLogMessage(0, 'Part ID 1: ' + this._partID1.toString())
      asLogger.addLogMessage(0, 'Gripper ID: ' + this._gripperID1.toString())
    }

    if ((resMode == ResultMode.GCP) || (resMode == ResultMode[ResultMode.GCP])) {
      asLogger.addLogMessage(0, 'Part ID 1: ' + this._partID1.toString())
      asLogger.addLogMessage(0, 'Part ID 2: ' + this._partID2.toString())
      asLogger.addLogMessage(0, 'Gripper ID: ' + this._gripperID1.toString())
    }

    this.copyRobotPose(3)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration !== null) {
      this._isCameraMoving = g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration.isCameraMoving_
    }

    this._validCMD = 1

    this.imgAcquired = function () {
      tracer.addMessage('-> XT: Image acquired' + timeTracker.getElapsedTime())

      let features = (this._featureMask & MASK_ID_1)

      this._enabledFeatures = features
      tracer.addMessage('<- XT: Image acquired' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_TOOLS_DONE
    }


    this.toolsDone = function (t) {
      tracer.addMessage('-> XT: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      let transformed = new Feature(0, 0, 0, 0)
      
      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            
            if (this.isFeatureEnabled(f) == true) {
              transformed = new Feature(0, 0, 0, 0)
              let shuttlingPose = this._shuttlingPose & MASK_ID_1
              let isCameraMoving = this._isCameraMoving & MASK_ID_1
              let isPartMoving = this._isPartMoving & MASK_ID_1

              transformed = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
              this._results[this._myIndex].data.push(transformed)
              if (this._results[this._myIndex].state >= 0) {
                this._results[this._myIndex].state = transformed.valid
              }
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
        }
      }
      tracer.addMessage('<- XT: Tools done ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XT: Compute result ' + timeTracker.getElapsedTime())

      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)      

      this.checkResultsState()      

      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        g_Graphics.ShowCrossHair = []

        let partInfo = g_LooukupTables.partLookup[this._partID1]        

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

            let targetID = camInfo['Feature_' + f]            
            
            if (targetID > 0) {             
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }           
          }
        }

        newRobPose = ComputeAlignMode_2(this._partID1, this._partID2, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid
        
      } else {
        retState = this._results.error
      }
      let resStr = ''
      if (retState > 0) {
        resStr = InSightFunctions.fnStringf(',%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)        
      }

      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)

      tracer.addMessage('<- XT: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XTS,<partID>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // XTS,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>,<Score>
  // SLMP: 4013
  XTS: function (myIndex, cmdString) { // Step ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 4013
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    var resMode = ResultMode.ABS

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }
    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 3; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    resMode = this._splittedCmd[2]
    if (!((resMode == ResultMode.ABS) || (resMode == ResultMode.OFF) || (resMode == ResultMode.FRAME) || (resMode == ResultMode.PICKED) ||
      (resMode == ResultMode[ResultMode.ABS]) || (resMode == ResultMode[ResultMode.OFF]) || (resMode == ResultMode[ResultMode.FRAME]) || (resMode == ResultMode[ResultMode.PICKED]))) {
      this._validCMD = ECodes.E_INVALID_ARGUMENT
      return
    }

    this.copyRobotPose(3)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    this._validCMD = 1

    this.toolsDone = function (t) {
      tracer.addMessage('-> XTS: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        let score = -1

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
            score = InSightFunctions.fnGetCellValue('Target.' + f + '.Pattern_Score')
            score = (Math.round(score * 1000) / 1000)
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push([transformed, score])
        }
      }
      tracer.addMessage('<- XTS: Tools done ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XTS: Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''
      let scores = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)

      this.checkResultsState()

      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        g_Graphics.ShowCrossHair = []

        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex][0].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex][0].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex][0].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex][0].thetaInDegrees
              scores = scores + InSightFunctions.fnStringf(',%.3f', this._results[c].data[resIndex][1])
              resIndex++
            }
          }
        }
        newRobPose = ComputeAlignMode_2(this._partID1, this._partID2, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid
      } else {
        retState = this._results.error
      }

      let resStr = ''
      if (retState > 0) {
        resStr = InSightFunctions.fnStringf(',%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)
        resStr = resStr + scores
      }

      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)

      tracer.addMessage('<- XTS: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XTLC,<partID>,<resultMode>,<lcheckMode>,<X>,<Y>,<Z>,<A>,<B>,<C> (1=ABS, 2=OFF, 3=FRAME, 4=GC, 5=GCP) (1=ABS, 2=DIFF, 3=REL)
  // XTLC,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 4014  
  XTLC: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 4014
    this._numArguments = 8
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    let types = [ARG_TYPES.INT, //partIDs
    [ARG_TYPES.DEF_RESULT_MODE, ['ABS', 'OFF', 'FRAME', 'GC', 'GCP']], // resultMode
    [ARG_TYPES.DEF_LCHECK_RESULT_MODE, ['ABS', 'DIFF', 'REL']], // LCResultMode
    ARG_TYPES.FLOAT, // X
    ARG_TYPES.FLOAT, // Y
    ARG_TYPES.FLOAT, // Z
    ARG_TYPES.FLOAT, // A
    ARG_TYPES.FLOAT, // B
    ARG_TYPES.FLOAT] // C

    var resMode = ResultMode.ABS
    var lcResMode = LCheckResultMode.ABS

    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }

    if (this.setIndex() != true) {
      return
    }   

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }
    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }
    
    let len = this._numArguments + this._hasIndex + 1
    for (let i = 4; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    resMode = this._splittedCmd[2]
    lcResMode = this._splittedCmd[3]
    lcResMode = getLCheckResultModeAsStr(lcResMode)

    if ((resMode == ResultMode.GC) || (resMode == ResultMode[ResultMode.GC])) {
      asLogger.addLogMessage(0, 'Part ID 1: ' + this._partID1.toString())
      asLogger.addLogMessage(0, 'Gripper ID: ' + this._gripperID1.toString())
    }

    if ((resMode == ResultMode.GCP) || (resMode == ResultMode[ResultMode.GCP])) {    
      asLogger.addLogMessage(0, 'Part ID 1: ' + this._partID1.toString())
      asLogger.addLogMessage(0, 'Part ID 2: ' + this._partID2.toString())
      asLogger.addLogMessage(0, 'Gripper ID: ' + this._gripperID1.toString())
    }

    this.copyRobotPose(4)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration !== null) {
      this._isCameraMoving = g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration.isCameraMoving_
    }

    if (isValidPartID(this._partID1) != true) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }

    if (g_LooukupTables.partLookup[this._partID1].FeatureIDs.length != 2) {
      this._validCMD = ECodes.E_COMBINATION_NOT_ALLOWED
      return
    }

    this._validCMD = 1

    this.imgAcquired = function () {
      tracer.addMessage('-> XTLC: Image acquired' + timeTracker.getElapsedTime())

      let features = (this._featureMask & MASK_ID_1)
      
      this._enabledFeatures = features
      tracer.addMessage('<- XTLC: Image acquired' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_TOOLS_DONE
    }


    this.toolsDone = function (t) {
      tracer.addMessage('-> XTLC: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      let transformed = new Feature(0, 0, 0, 0)
      let transformed2 = new Feature(0, 0, 0, 0)

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {

            if (this.isFeatureEnabled(f) == true) {
              transformed = new Feature(0, 0, 0, 0)
              let shuttlingPose = this._shuttlingPose & MASK_ID_1
              let isCameraMoving = this._isCameraMoving & MASK_ID_1
              let isPartMoving = this._isPartMoving & MASK_ID_1

              transformed = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
              this._results[this._myIndex].data.push(transformed)
              if (this._results[this._myIndex].state >= 0) {
                this._results[this._myIndex].state = transformed.valid
              }
            }      
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
        }
      }
      tracer.addMessage('<- XTLC: Tools done ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XTLC: Compute result ' + timeTracker.getElapsedTime())

      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)
      
      this.checkResultsState()

      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        g_Graphics.ShowCrossHair = []

        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]

          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

            let targetID = camInfo['Feature_' + f]

            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }
          }
        }

        newRobPose = ComputeAlignMode_2(this._partID1, this._partID2, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid
        
      } else {
        retState = this._results.error
      }
      let retDist = 0
      let dists
      if (retState > 0) {
        
        dists = getPartsFeatureDistance(this._partID1, this._gripperID1)
       
        if (dists['valid'] === true) {       
          retDist = dists[lcResMode]
        } else {
          retState = dists['valid']
        }
      }
      let resStr = ''
      if (retState > 0) {
        let jdg = 0
        if (dists['judge'] === true) jdg = 1;
        resStr = InSightFunctions.fnStringf(',%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d', newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX, retDist, jdg)
      }
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)

      tracer.addMessage('<- XTLC: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XTMP,<partIDs>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // XTMP,<Status>,<partID>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 4015
  XTMP: function (myIndex, cmdString) { // Step ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 4015
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    let types = [ARG_TYPES.INT, //partIDs
      [ARG_TYPES.DEF_RESULT_MODE, ['ABS', 'OFF', 'FRAME']], // resultMode
    ARG_TYPES.FLOAT, // X
    ARG_TYPES.FLOAT, // Y
    ARG_TYPES.FLOAT, // Z
    ARG_TYPES.FLOAT, // A
    ARG_TYPES.FLOAT, // B
    ARG_TYPES.FLOAT] // C

    var resMode = ResultMode.ABS
    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }

    if (this.setIndex() != true) {
      return
    }

    this.decodeIDsfromIndex()
    this.getNextID()
    this._partID1 = this._decodedIDs[this._decodedIDs[0]]
    resMode = this._splittedCmd[2]   

    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }   

    this.copyRobotPose(3)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration !== null) {
      this._isCameraMoving = g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration.isCameraMoving_
    }

    this._validCMD = 1

    /*   
    this.imgAcquired = function () {
      tracer.addMessage('-> XTMP: Image acquired' + timeTracker.getElapsedTime())
      let features = (this._featureMask & MASK_ID_1)
   
      this._enabledFeatures = features
      tracer.addMessage('<- XTMP: Image acquired' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_TOOLS_DONE
    }
    */

    this.toolsDone = function (t) {
      tracer.addMessage('-> XTMP: Tools done ' + timeTracker.getElapsedTime())

      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }      

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }                        
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push(transformed)
        }
      }
      tracer.addMessage('<- XTMP: Tools done ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XTMP: Compute result ' + timeTracker.getElapsedTime())

      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)
      this.checkResultsState()

      let retState = ECodes.E_UNSPECIFIED

      if (this._results.error == ECodes.E_NO_ERROR) {
        g_Graphics.ShowCrossHair = []

        let partInfo = g_LooukupTables.partLookup[this._partID1]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]

          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

            let targetID = camInfo['Feature_' + f]

            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }
          }
        }

        newRobPose = ComputeAlignMode_2(this._partID1, this._partID2, this._gripperID1, resMode, this._robotPose)
        retState = newRobPose.valid        
      } else {
        retState = this._results.error
      }
      let resStr = ''
      if (retState > 0) {
        resStr = InSightFunctions.fnStringf(',%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', this._decodedIDs[this._decodedIDs[0]], newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)        
      }

      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], retState, resStr)

      if (retState <= 0) {
        if (this.getNextID() == true) {
          plcString = 'REDO_FROM_EXECUTE'
          this._partID1 = this._decodedIDs[this._decodedIDs[0]]
          this.resetResultStructure()
          this.fillCmdInfoWithPartID()
        }
      }

      tracer.addMessage('<- XTMP: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // XT2,<partIDs>,<resultMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // XT2,<Status>,<X1>,<Y1>,<Z1>,<A1>,<B1>,<C1>,<X2>,<Y2>,<Z2>,<A2>,<B2>,<C2>
  // SLMP: 4016
  XT2: function (myIndex, cmdString) { // Step ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 4016
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._logImageType = 'LogImage.IsProductionImage'
    let types = [ARG_TYPES.INT, //partIDs
    [ARG_TYPES.DEF_RESULT_MODE, ['ABS', 'OFF', 'FRAME']], // resultMode
    ARG_TYPES.FLOAT, // X
    ARG_TYPES.FLOAT, // Y
    ARG_TYPES.FLOAT, // Z
    ARG_TYPES.FLOAT, // A
    ARG_TYPES.FLOAT, // B
    ARG_TYPES.FLOAT] // C
    var resMode = ResultMode.ABS

    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }

    if (this.setIndex() != true) {
      return
    }

    this.decodeIDsfromIndex()
    this.getNextID()
    this._partID1 = this._decodedIDs[1]
    this._partID2 = this._decodedIDs[2]

    resMode = this._splittedCmd[2]   


    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }
    if (!g_Parts.hasOwnProperty(this._partID2)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }
    
    if (this._partID2 > 0) {
      this._computeBothParts = 1
    }    
    
    if ((resMode == ResultMode.GC) || (resMode == ResultMode[ResultMode.GC])) {
      asLogger.addLogMessage(0, 'Part ID 1: ' + this._partID1.toString())
      asLogger.addLogMessage(0, 'Gripper ID: ' + this._gripperID1.toString())
    }

    this.copyRobotPose(3)

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }

    if (g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration !== null) {
      this._isCameraMoving = g_Calibrations[(this._shuttlingPose & MASK_ID_1)].calibration.isCameraMoving_
    }

    this._validCMD = 1

    this.imgAcquired = function () {
      tracer.addMessage('-> XT2: Image acquired' + timeTracker.getElapsedTime())

      let features = (this._featureMask & MASK_ID_1)

      if (this._computeBothParts == 1) {

        features = features | ((this._featureMask & MASK_ID_2) >> SHIFT_ID_2)
      }

      this._enabledFeatures = features
      tracer.addMessage('<- XT2: Image acquired' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_TOOLS_DONE
    }


    this.toolsDone = function (t) {
      tracer.addMessage('-> XT2: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      let transformed = new Feature(0, 0, 0, 0)
      let transformed2 = new Feature(0, 0, 0, 0)

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {

            if ((this._featureMask & MASK_ID_1) & (0x01 << (f - 1))) {              
              transformed = new Feature(0, 0, 0, 0)
              let shuttlingPose = this._shuttlingPose & MASK_ID_1
              let isCameraMoving = this._isCameraMoving & MASK_ID_1
              let isPartMoving = this._isPartMoving & MASK_ID_1

              transformed = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
              this._results[this._myIndex].data.push(transformed)
              if (this._results[this._myIndex].state >= 0) {
                this._results[this._myIndex].state = transformed.valid
              }
            }

            if (this._computeBothParts == 1) {
              if (((this._featureMask & MASK_ID_2) >> SHIFT_ID_2) & (0x01 << (f - 1))) {
                transformed2 = new Feature(0, 0, 0, 0)
                let shuttlingPose = (this._shuttlingPose & MASK_ID_2) >> SHIFT_ID_2
                let isCameraMoving = (this._isCameraMoving & MASK_ID_2) >> SHIFT_ID_2
                let isPartMoving = (this._isPartMoving & MASK_ID_2) >> SHIFT_ID_2

                transformed2 = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
                this._results[this._myIndex].data.push(transformed2)
                if (this._results[this._myIndex].state >= 0) {
                  this._results[this._myIndex].state = transformed2.valid
                }
              }
            }

          } else {
            let emptyData = new Feature(0, 0, 0, g_CurrentFeatures[f].valid)
            this._results[this._myIndex].data.push(emptyData)            
          }
        }
      }
      tracer.addMessage('<- XT2: Tools done ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> XT2: Compute result ' + timeTracker.getElapsedTime())

      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)
      let newRobPose2 = new RobotPose(0, 0, 0, 0, 0, 0, 0)

      g_Graphics.ShowCrossHair = []

      let partInfo = g_LooukupTables.partLookup[this._partID1]

      for (let c = 1; c <= MAX_CAMERAS; c++) {
        let resIndex = 0
        let camInfo = partInfo['Cam_' + c]

        for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

          let targetID = camInfo['Feature_' + f]

          if (targetID > 0) {
            g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
            g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
            g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
            g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
            resIndex++
          }

          if (this._computeBothParts == 1) {
            let partInfo2 = g_LooukupTables.partLookup[this._partID2]
            let camInfo2 = partInfo2['Cam_' + c]
            let targetID2 = camInfo2['Feature_' + f]
            if (targetID2 > 0) {
              g_RuntimeFeatures[targetID2].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID2].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID2].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID2].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }
          }
        }
      }



      let res = checkPartRuntimeFeatures(this._partID1)

      if (res == ECodes.E_NO_ERROR) {
        newRobPose = ComputeAlignMode_2(this._partID1, this._partID2, this._gripperID1, resMode, this._robotPose)
      } else {
        newRobPose.valid = res
      }

      if (this._computeBothParts == 1) {
        res = checkPartRuntimeFeatures(this._partID2)
        if (res == ECodes.E_NO_ERROR) {
          newRobPose2 = ComputeAlignMode_2(this._partID2, this._partID2, this._gripperID1, resMode, this._robotPose)
        } else {
          newRobPose2.valid = res
        }
      }

      let resStr = ''
      if (newRobPose.valid > 0) {        
        resStr = InSightFunctions.fnStringf(',%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', newRobPose.valid, newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)
      } else {
        resStr = InSightFunctions.fnStringf(',%d', newRobPose.valid)
      }

      if (this._computeBothParts == 1) {
        if (newRobPose2.valid > 0) {
          resStr = InSightFunctions.fnStringf('%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', resStr, newRobPose2.valid, newRobPose2.x, newRobPose2.y, newRobPose2.z, newRobPose2.thetaZ, newRobPose2.thetaY, newRobPose2.thetaX)
        } else {
          resStr = InSightFunctions.fnStringf('%s,%d', resStr, newRobPose2.valid)
        }
      }   

      plcString = InSightFunctions.fnStringf('%s%s', this._splittedCmd[0], resStr)

      tracer.addMessage('<- XT2: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }
  },

  // CP
  // CP, <Status>
  // SLMP: 5010
  CP: function (myIndex, cmdString) { // Camera ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 5010
    this._numArguments = 0
    this._hasIndex = 0
    this._onlyForMaster = 1

    this._validCMD = 1

    this._index = this._myIndex

    this.execute = function (t) {
      tracer.addMessage('-> CP: Execute')

      for (let i in g_RuntimeFeatures) {
        g_RuntimeFeatures[i].reset()
      }

      this._results[this._myIndex].state = 1
      this._results[this._myIndex].isValid = 1
      tracer.addMessage('<- CP: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> CP: Compute result')
      let plcString = ''
      let state = 1
      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], state)
      tracer.addMessage('<- CP: Compute result')
      return plcString
    }
  },

  // LF, <StepID>, <ProductID>,[<X>,<Y>,<Z>,<A>,<B>,<C>]
  // LF, <Status>, <Token>, <ProductID>
  // SLMP: 5011
  LF: function (myIndex, cmdString) { // Step ID
    tracer.addMessage('-> LF init ' + timeTracker.getElapsedTime())
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 5011
    this._numArguments = 1
    this._hasIndex = 1
    this._useAsStepID = 1
    this._logImageType = 'LogImage.IsProductionImage'

    if (this.checkLengthAndSetIndex(1, RECIPE_MAX_STEPS) != true) {
      return
    }
    if (!g_Steps.hasOwnProperty(this._index)) {
      this._validCMD = ECodes.E_INDEX_OUT_OF_RANGE
      return
    }
    let len = this._numArguments + this._hasIndex + 1
    for (let i = 1; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    var productID = parseFloat(this._splittedCmd[2])
    if (this._splittedCmd.length >= 9) {
      this._robotPose.x = parseFloat(this._splittedCmd[3])
      this._robotPose.y = parseFloat(this._splittedCmd[4])
      this._robotPose.z = parseFloat(this._splittedCmd[5])
      this._robotPose.thetaZ = parseFloat(this._splittedCmd[6])
      this._robotPose.thetaY = parseFloat(this._splittedCmd[7])
      this._robotPose.thetaX = parseFloat(this._splittedCmd[8])
      this._robotPose.valid = 1
    }
    if (this.fillCmdInfoWithStepID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    this._validCMD = 1

    this.computeTotalResult = function () {
      tracer.addMessage('-> LF: Compute result ' + timeTracker.getElapsedTime())
      let plcString = ''

      this.checkResultsState()
      let retState = ECodes.E_UNSPECIFIED
      if (this._results.error == ECodes.E_NO_ERROR) {
        let partInfo = g_LooukupTables.stepLookup[this._index]

        for (let c = 1; c <= MAX_CAMERAS; c++) {
          let resIndex = 0
          let camInfo = partInfo['Cam_' + c]
          for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
            let targetID = camInfo['Feature_' + f]
            if (targetID > 0) {
              g_RuntimeFeatures[targetID].valid = this._results[c].data[resIndex].valid
              g_RuntimeFeatures[targetID].x = this._results[c].data[resIndex].x
              g_RuntimeFeatures[targetID].y = this._results[c].data[resIndex].y
              g_RuntimeFeatures[targetID].thetaInDegrees = this._results[c].data[resIndex].thetaInDegrees
              resIndex++
            }
          }
        }
        retState = 1
      } else {
        retState = this._results.error
      }

      plcString = InSightFunctions.fnStringf('%s,%d,0,%d', this._splittedCmd[0], retState, productID)

      tracer.addMessage('<- LF: Compute result ' + timeTracker.getElapsedTime())
      return plcString
    }

    tracer.addMessage('<- LF init ' + timeTracker.getElapsedTime())
  },

  // TP,<partID>,<AlignMode>
  // TP,<Status>
  // SLMP: 5012
  TP: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 5012
    this._numArguments = 1
    this._hasIndex = 1
    this._useAsPartID = 1
    this._onlyForMaster = 1

    if (this.checkLengthAndSetIndex(1, RECIPE_MAX_PARTS) != true) {
      return
    }

    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }

    let len = this._numArguments + this._hasIndex + 1
    for (let i = 2; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    this._alignMode = parseInt(this._splittedCmd[2])

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> TP: Execute')
      let validCnt = 0
      for (var f = 0; f < g_Parts[this._index]['runtimeFeatures'].length; f++) {
        if (g_Parts[this._index]['runtimeFeatures'][f].valid > 0) {
          g_Parts[this._index]['trainedFeatures'][f][this._gripperID1].x = g_Parts[this._index]['runtimeFeatures'][f].x
          g_Parts[this._index]['trainedFeatures'][f][this._gripperID1].y = g_Parts[this._index]['runtimeFeatures'][f].y
          g_Parts[this._index]['trainedFeatures'][f][this._gripperID1].thetaInDegrees = g_Parts[this._index]['runtimeFeatures'][f].thetaInDegrees
          g_Parts[this._index]['trainedFeatures'][f][this._gripperID1].valid = g_Parts[this._index]['runtimeFeatures'][f].valid

          validCnt++
        }
      }

      for (let c = 1; c <= MAX_CAMERAS; c++) {
        if (c != this._myIndex) {
          this._results[c].isNeeded = 0
        }
      }

      this._results[this._myIndex].isValid = 1

      if (validCnt == g_Parts[this._index]['runtimeFeatures'].length) {
        this._results[this._myIndex].state = 1
      } else {
        this._results[this._myIndex].state = ECodes.E_PART_NOT_ALL_FEATURES_LOCATED
      }
      tracer.addMessage('<- TP: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> TP: Compute result')
      let plcString = ''
      let state = -1
      tracer.addMessage(this._results)
      if (this._results[this._myIndex].isValid > 0) {
        state = this._results[this._myIndex].state
        if (state > 0) {
          InSightFunctions.fnSetEvent(83)
        }
      }

      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], state)
      tracer.addMessage('<- TP: Compute result')
      return plcString
    }
  },

  // TPR,<partID>,<AlignMode>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // TPR,<Status>
  // SLMP: 5013
  TPR: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 5013
    this._numArguments = 7
    this._hasIndex = 1
    this._useAsPartID = 1
    this._onlyForMaster = 1

    if (this.checkLengthAndSetIndex(1, RECIPE_MAX_PARTS) != true) {
      return
    }

    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }
    let len = this._numArguments + this._hasIndex + 1
    for (let i = 2; i < len; i++) {
      if (isNaN(this._splittedCmd[i])) {
        this._validCMD = ECodes.E_INVALID_ARGUMENT
        return
      }
    }

    this._alignMode = parseInt(this._splittedCmd[2])

    this._robotPose.x = parseFloat(this._splittedCmd[3])
    this._robotPose.y = parseFloat(this._splittedCmd[4])
    this._robotPose.z = parseFloat(this._splittedCmd[5])
    this._robotPose.thetaZ = parseFloat(this._splittedCmd[6])
    this._robotPose.thetaY = parseFloat(this._splittedCmd[7])
    this._robotPose.thetaX = parseFloat(this._splittedCmd[8])
    this._robotPose.valid = 1

    if (g_Parts.hasOwnProperty(this._index) == false) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }

    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }

    for (let c = 1; c <= MAX_CAMERAS; c++) {
      if (c == this._myIndex) {
        this._results[c].isNeeded = 1
      } else {
        this._results[c].isNeeded = 0
      }
    }
    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> TPR: Execute')

      g_Parts[this._index].trainedRobotPose[this._gripperID1].x = this._robotPose.x
      g_Parts[this._index].trainedRobotPose[this._gripperID1].y = this._robotPose.y
      g_Parts[this._index].trainedRobotPose[this._gripperID1].z = this._robotPose.z
      g_Parts[this._index].trainedRobotPose[this._gripperID1].thetaZ = this._robotPose.thetaZ
      g_Parts[this._index].trainedRobotPose[this._gripperID1].thetaY = this._robotPose.thetaY
      g_Parts[this._index].trainedRobotPose[this._gripperID1].thetaX = this._robotPose.thetaX
      g_Parts[this._index].trainedRobotPose[this._gripperID1].valid = this._robotPose.valid

      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1

      for (let c = 1; c <= MAX_CAMERAS; c++) {
        if (c != this._myIndex) {
          this._results[c].isNeeded = 0
        }
      }

      tracer.addMessage('<- TPR: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> TPR: Compute result')
      let plcString = ''
      let state = 1
      if (this._results[this._myIndex].isValid > 0) {
        state = this._results[this._myIndex].state
        InSightFunctions.fnSetEvent(83)
      }
      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], state)
      tracer.addMessage('<- TPR: Compute result')
      return plcString
    }
  },

  // GP,<partID>,<AlignMode>,<ResultMode>, [current motion pose]
  // GP,<Status>,<X>,<Y>,<Z>,<A>,<B>,<C>
  // SLMP: 5014
  GP: function (myIndex, cmdString) { // Part ID
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 5014
    this._numArguments = 1
    this._hasIndex = 1
    this._useAsPartID = 1
    this._onlyForMaster = 1

    var resMode = ResultMode.ABS

    if (this.checkLengthAndSetIndex(1, RECIPE_MAX_PARTS) != true) {
      return
    }

    if (!g_Parts.hasOwnProperty(this._partID1)) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }
    this._alignMode = parseInt(this._splittedCmd[2])

    resMode = this._splittedCmd[3]    
    if (isValidResultMode(resMode) != true) {
      this._validCMD = ECodes.E_INVALID_RESULT_MODE
      return
    }

    if (this._splittedCmd.length >= 10) {
      for (let i = 4; i < 10; i++) {
        if (isNaN(this._splittedCmd[i])) {
          this._validCMD = ECodes.E_INVALID_ARGUMENT
          return
        }
      }

      this._robotPose.x = parseFloat(this._splittedCmd[4])
      this._robotPose.y = parseFloat(this._splittedCmd[5])
      this._robotPose.z = parseFloat(this._splittedCmd[6])
      this._robotPose.thetaZ = parseFloat(this._splittedCmd[7])
      this._robotPose.thetaY = parseFloat(this._splittedCmd[8])
      this._robotPose.thetaX = parseFloat(this._splittedCmd[9])
      this._robotPose.valid = 1
    }
    
    if (this.fillCmdInfoWithPartID() !== true) {
      return
    }
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }

    if ((resMode == ResultMode.GC) || (resMode == ResultMode[ResultMode.GC])) {
      asLogger.addLogMessage(0, 'Part ID 1: ' + this._partID1.toString())
      asLogger.addLogMessage(0, 'Gripper ID: ' + this._gripperID1.toString())
    }

    if ((resMode == ResultMode.GCP) || (resMode == ResultMode[ResultMode.GCP])) {
      asLogger.addLogMessage(0, 'Part ID 1: ' + this._partID1.toString())
      asLogger.addLogMessage(0, 'Part ID 2: ' + this._partID2.toString())
      asLogger.addLogMessage(0, 'Gripper ID: ' + this._gripperID1.toString())
    }

    if (this._isCameraMoving && (this._alignMode==1)) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
      return
    }

    this._validCMD = 1

    this.execute = function (t) {
      tracer.addMessage('-> GP: Execute')
      let validCnt = 0
      for (var f = 0; f < g_Parts[this._partID1]['runtimeFeatures'].length; f++) {
        if (g_Parts[this._partID1]['runtimeFeatures'][f].valid > 0) {
          validCnt++
        }
      }

      for (let c = 1; c <= MAX_CAMERAS; c++) {
        if (c != this._myIndex) {
          this._results[c].isNeeded = 0
        }
      }
      this._results[this._myIndex].isValid = 1
      if (validCnt == g_Parts[this._partID1]['runtimeFeatures'].length) {
        this._results[this._myIndex].state = 1
      } else {
        this._results[this._myIndex].state = ECodes.E_PART_NOT_ALL_FEATURES_LOCATED
      }

      tracer.addMessage('<- GP: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> GP: Compute result')
      let plcString = ''
      let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)
      if (this._results[this._myIndex].isValid > 0) {
        if (this._alignMode == 1) {
          if (this._robotPose.valid > 0) {
            newRobPose = ComputeAlignMode_1(this._partID1, this._gripperID1, resMode, this._robotPose)
          }
        } else if (this._alignMode == 2) {
          if (this._robotPose.valid > 0) {
            newRobPose = ComputeAlignMode_2(this._partID1, this._partID2, this._gripperID1, resMode, this._robotPose)
          }
        } else if (this._alignMode == 3) {
          if (this._robotPose.valid > 0) {
            newRobPose = ComputeAlignMode_3(this._partID1, this._partID2, this._gripperID1, resMode, this._robotPose)
          }
        } else {
          newRobPose.valid = ECodes.E_NOT_SUPPORTED
        }
      }
      let resStr = ''
      if (newRobPose.valid > 0) {
        resStr = InSightFunctions.fnStringf(',%.3f,%.3f,%.3f,%.3f,%.3f,%.3f', newRobPose.x, newRobPose.y, newRobPose.z, newRobPose.thetaZ, newRobPose.thetaY, newRobPose.thetaX)
      }
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], newRobPose.valid, resStr)
      tracer.addMessage('<- GP: Compute result')
      return plcString
    }
  },

  // LC,<partID>,<lcheckMode>
  // LC, <Status>
  // SLMP: 5015
  LC: function (myIndex, cmdString) { 
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 5015
    this._numArguments = 1
    this._hasIndex = 1
    this._useAsPartID = 1
    this._onlyForMaster = 1

    let types = [ARG_TYPES.INT, //partIDs
    [ARG_TYPES.DEF_LCHECK_RESULT_MODE, ['ABS', 'DIFF', 'REL']]] // LCResultMode

    var lcResMode = LCheckResultMode.ABS

    if (this.checkLength() != true) {
      return
    }

    if (this.checkArgTypes(types) != true) {
      return
    }

    if (this.setIndex() != true) {
      return
    }   

    if (this.checkLengthAndSetIndex(0, RECIPE_MAX_STEPS) != true) {
      return
    }

    lcResMode = this._splittedCmd[2]
    lcResMode = getLCheckResultModeAsStr(lcResMode)

    if (isValidPartID(this._partID1) != true) {
      this._validCMD = ECodes.E_INVALID_PART_ID
      return
    }

    if (g_LooukupTables.partLookup[this._partID1].FeatureIDs.length != 2) {
      this._validCMD = ECodes.E_COMBINATION_NOT_ALLOWED
      return
    }

    this._validCMD = 1

    this._index = this._myIndex

    this.execute = function (t) {
      tracer.addMessage('-> LC: Execute')
      this._results[this._myIndex].state = 1
      this._results[this._myIndex].isValid = 1
      tracer.addMessage('<- LC: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> LC: Compute result')

      let state = 1
      let resStr = ''
      let retDist = 0
      let dists = getPartsFeatureDistance(this._partID1, this._gripperID1)

      if (dists['valid'] === true) {
        retDist = dists[lcResMode]

        let jdg = 0
        if (dists['judge'] === true) jdg = 1;
        
        resStr = InSightFunctions.fnStringf(',%.3f,%d',retDist, jdg)
      } else {
        state = dists['valid']
      }

      let plcString = ''
      
      plcString = InSightFunctions.fnStringf('%s,%d%s', this._splittedCmd[0], state, resStr)
      tracer.addMessage('<- LC: Compute result')
      return plcString
    }
  },

  //  PID,<ProductID>
  //  PID,<Status>
  // SLMP: 5050
  PID: function (myIndex, cmdString) { // 
    CommandBase.call(this, myIndex, cmdString)
    this._slmpCode = 5050
    this._numArguments = 1
    this._hasIndex = 1
    let Pid = this._splittedCmd[1]
    this._splittedCmd.push(this._splittedCmd[1])    
    this._splittedCmd[1] = '0'
    this._splittedCmd[2] = Pid 

    if (this.checkLengthAndSetIndex(0, MAX_CAMERAS) != true) {
      return
    }

    if (this.fillCmdInfoWithCameraID(0, 0, 0, 0) !== true) {
      return
    }
    
    this.pid = this._splittedCmd[2]    

    this._validCMD = 1    

    this.execute = function (t) {
      tracer.addMessage('-> PID: Execute')

      writeCellValue("LogImage.ProductID" ,this.pid)

      this._results[this._myIndex].data.push({ })
      this._results[this._myIndex].state = 1
      this._results[this._myIndex].isValid = 1
      tracer.addMessage('<- PID: Execute')
      return States.WAITING_FOR_SLAVE_RESULT
    }

    this.computeTotalResult = function () {
      tracer.addMessage('-> PID: Compute result')
      let plcString = ''
      let retState = ECodes.E_UNSPECIFIED
      this.checkResultsState()

      if (this._results.error == ECodes.E_NO_ERROR) {
        retState = 1
      } else {
        retState = this._results.error
      }

      plcString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], retState)
      tracer.addMessage('<- PID: Compute result')
      return plcString
    }
  }
}

//* ***************************************************************************/
//* ***************************************************************************/
function SlaveBase(myIndex, cmdString) {
  this._myIndex = myIndex

  this._cmdString = cmdString
  this._splittedCmd = cmdString.split(',')

  this._enabledFeatures = 0
  this._featureMask = parseInt(this._splittedCmd[1])
  this._exposureSet = parseInt(this._splittedCmd[2])

  this._shuttlingPose = parseInt(this._splittedCmd[3])
  this._isCameraMoving = parseInt(this._splittedCmd[4]) || g_Settings.IsRobotMounted

  this._isPartMoving = parseInt(this._splittedCmd[5])
  this._computeBothParts = 0

  this._numArguments = -1

  this._index = -1

  this._validCMD = 1
  this._gripperID1 = 0

  this._robotPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)
  this._results = {}
  this._results.error = ECodes.E_NO_ERROR

  this._results[this._myIndex] = new Result()
  this._results[this._myIndex].isNeeded = 1

  this._logImageType = ''
};
SlaveBase.prototype.isValid = function () {
  return this._validCMD
}
SlaveBase.prototype.execute = function (t) {
  tracer.addMessage('-> Execute ' + timeTracker.getElapsedTime())

  this._enabledFeatures = 0

  setFeatureAcqSettings(this._exposureSet - 1)

  if (this._logImageType.length > 2) {
    InSightFunctions.fnSetCellValue(this._logImageType, 1)
  }

  if (t.triggerMode == 32) {
    InSightFunctions.fnSetEvent(32)
  }

  tracer.addMessage('<- Execute ' + timeTracker.getElapsedTime())
  return States.WAITING_FOR_IMAGE_ACQUIRED
}

SlaveBase.prototype.imgAcquired = function () {
  tracer.addMessage('-> Image acquired ' + timeTracker.getElapsedTime())

  this._enabledFeatures = this._featureMask

  tracer.addMessage('<- Image acquired ' + timeTracker.getElapsedTime())

  return States.WAITING_FOR_TOOLS_DONE
}
SlaveBase.prototype.toolsDone = function (t) {
  tracer.addMessage('-> Tools done ' + timeTracker.getElapsedTime())

  if (this._logImageType.length > 2) {
    InSightFunctions.fnSetCellValue(this._logImageType, 0)
  }

  this._results[this._myIndex]['isValid'] = 1

  for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
    let transformed = new Feature(0, 0, 0, 0)

    if (this.isFeatureEnabled(f) == true) {
      if (g_CurrentFeatures[f].valid > 0) {

        transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
        if (this._results[this._myIndex].state >= 0) {
          if (this._results[this._myIndex].state >= 0) {
            this._results[this._myIndex].state = transformed.valid
          }
        }

      } else {
        this._results[this._myIndex].state = g_CurrentFeatures[f].valid
      }
      this._results[this._myIndex].data.push(transformed)
    }
  }

  tracer.addMessage('<- Tools done ' + timeTracker.getElapsedTime())

  return States.WAITING_FOR_NEW_COMMAND
}
SlaveBase.prototype.computeTotalResult = function () {
  tracer.addMessage('-> Compute result ' + timeTracker.getElapsedTime())
  let resString = ''
  let errCnt = 0
  let errCode

  if (this._results['1'].isValid > 0) {
    if (this._results['1'].state <= 0) {
      errCnt++
      errCode = this._results['1'].state
    }
  }

  let state = ECodes.E_INTERNAL_ERROR
  if (errCnt == 0) {
    state = 1 // No error
  } else {
    state = errCode
  }

  resString = InSightFunctions.fnStringf('%s,%d', this._splittedCmd[0], state)

  tracer.addMessage('<-  Compute result ' + timeTracker.getElapsedTime())
  return resString
}
SlaveBase.prototype.isFeatureEnabled = function (featureID) {
  return !!(this._enabledFeatures & (1 << (featureID - 1)))
}
SlaveBase.prototype.copyRobotPose = function (start) {
  this._robotPose.x = parseFloat(this._splittedCmd[start])
  this._robotPose.y = parseFloat(this._splittedCmd[start + 1])
  this._robotPose.z = parseFloat(this._splittedCmd[start + 2])
  this._robotPose.thetaZ = parseFloat(this._splittedCmd[start + 3])
  this._robotPose.thetaY = parseFloat(this._splittedCmd[start + 4])
  this._robotPose.thetaX = parseFloat(this._splittedCmd[start + 5])
  this._robotPose.valid = 1
}

var slaveCommands = {
  myIndex: 1,
  GS: function (myIndex, cmdString) {
    tracer.addMessage('-> GS init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    tracer.addMessage('<- GS init ' + +timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> GS: Execute')
      let major = InSightFunctions.fnGetCellValue('Version.Major')
      let minor = InSightFunctions.fnGetCellValue('Version.Minor')
      let subMinor = InSightFunctions.fnGetCellValue('Version.SubMinor')
      let jobName = InSightFunctions.fnGetCellValue('Internal.Recipe.JobName')

      this._results[this._myIndex].data.push({ 'Major': major, 'Minor': minor, 'SubMinor': subMinor, 'JobName': jobName })
      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1
      tracer.addMessage('<- GS: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  GV: function (myIndex, cmdString) {
    tracer.addMessage('-> GV init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)

    tracer.addMessage('<- GV init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> GV: Execute')
      let major = InSightFunctions.fnGetCellValue('Version.Major')
      let minor = InSightFunctions.fnGetCellValue('Version.Minor')
      let subMinor = InSightFunctions.fnGetCellValue('Version.SubMinor')
      this._results[this._myIndex].data.push({ 'Major': major, 'Minor': minor, 'SubMinor': subMinor })
      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1
      tracer.addMessage('<- GV: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  TM: function (myIndex, cmdString) {
    tracer.addMessage('-> TM init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    var triggerMode = parseInt(this._splittedCmd[6])
    tracer.addMessage('<- TM init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> TM: Execute')
      tracer.addMessage('Triggermode ' + triggerMode)
      this._results[this._myIndex].isValid = 1
      if ((triggerMode == 0) || (triggerMode == 1)) {
        if (triggerMode == 0) {
          triggerMode = 32
        } else {
          triggerMode = 0
        }

        InSightFunctions.fnSetCellValue('TriggerMode', triggerMode)
        this._results[this._myIndex].state = 1
      } else {
        this._results[this._myIndex].state = ECodes.E_INVALID_ARGUMENT
      }

      tracer.addMessage('<- TM: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  HEB: function (myIndex, cmdString) {
    tracer.addMessage('-> HEB init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    tracer.addMessage('<- HEB init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> HEB: Execute')
      InSightFunctions.fnSetCellValue('HECalibration.IsCameraMoving', this._isCameraMoving)
      
      g_HECalibrationSettings = new HECalibrationSettings()
      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1
      tracer.addMessage('<- HEB: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  HE: function (myIndex, cmdString) {
    tracer.addMessage('-> HE init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this.copyRobotPose(7)
    var featureID = getFeatureIDsFromFeatureMask(this._featureMask)[0]
    if (featureID == 0) {
      featureID = 1
    }
    this.execute = function (t) {
        tracer.addMessage('-> Execute ' + timeTracker.getElapsedTime())

        this._enabledFeatures = 0

        setFeatureAcqSettings(this._exposureSet - 1)

        if (this._logImageType.length > 2) {
            InSightFunctions.fnSetCellValue(this._logImageType, 1)
        }

        if (t.triggerMode == 32) {
            InSightFunctions.fnSetEvent(32)
        }
        if (g_HECalibrationSettings != null) {
            if (g_HECalibrationSettings.calibrationID == -1) {
                g_HECalibrationSettings.calibrationID = this._shuttlingPose
                g_Calibrations[this._shuttlingPose].deleteCalibrationDataFile()
                t.onReloadHeCalibrations()
                g_Calibrations[this._shuttlingPose] = new Calibration(this._shuttlingPose)
                g_Calibrations[this._shuttlingPose]['calibrationData']['isMoving'] = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
                InSightFunctions.fnSetCellValue('HECalibration.' + this._shuttlingPose + '.IsCameraMoving', g_Calibrations[this._shuttlingPose]['calibrationData']['isMoving'])
            } else if (g_HECalibrationSettings.calibrationID != this._shuttlingPose) {
                this._validCMD.ECodes.E_NOT_SUPPORTED
            }
        } else {
            this._validCMD = ECodes.E_NO_START_COMMAND
        }
        tracer.addMessage('<- HE init ' + timeTracker.getElapsedTime())
        tracer.addMessage('<- Execute ' + timeTracker.getElapsedTime())
        return States.WAITING_FOR_IMAGE_ACQUIRED
    }
    this.toolsDone = function (t) {
      tracer.addMessage('-> HE: Tools done')
      g_Graphics['ShowCalibrationPoints_' + this._shuttlingPose] = 1
      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')
      let targetTrained = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained')
      this._results[this._myIndex].isValid = 1

      if (targetTrained > 0) {
        if (targetValid > 0) {
          var targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          var targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          var targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

          g_Calibrations[this._shuttlingPose].calibrationData.targetX.push(targetX)
          g_Calibrations[this._shuttlingPose].calibrationData.targetY.push(targetY)
          g_Calibrations[this._shuttlingPose].calibrationData.targetTheta.push(targetAngle)
          g_Calibrations[this._shuttlingPose].calibrationData.targetValid.push(targetValid)
          g_Calibrations[this._shuttlingPose].calibrationData.robotX.push(this._robotPose.x)
          g_Calibrations[this._shuttlingPose].calibrationData.robotY.push(this._robotPose.y)
          g_Calibrations[this._shuttlingPose].calibrationData.robotTheta.push(this._robotPose.thetaZ)

          g_Calibrations[this._shuttlingPose].calibrationData.count = g_Calibrations[this._shuttlingPose].calibrationData.targetX.length
          if (this._results[this._myIndex].state >= 0) {
            this._results[this._myIndex].state = 1
          }
        } else {
          this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_FOUND
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
      }

      tracer.addMessage('<- HE: Tools done')
      return States.WAITING_FOR_SLAVE_RESULT
    }
  },
  HEE: function (myIndex, cmdString) {
    tracer.addMessage('-> HEE init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)

    if (g_HECalibrationSettings == null) {
      this._validCMD = ECodes.E_NO_START_COMMAND
      return
    }
    var selectedCalibration = g_HECalibrationSettings.calibrationID

    tracer.addMessage('<- HEE init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> HEE: Execute')
      g_Graphics['ShowCalibrationPoints_' + selectedCalibration] = 1
      this._results[this._myIndex].isValid = 1
      let isMoving = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
      var valid = t.myCalibrations.CheckData(g_Calibrations[selectedCalibration].calibrationData, isMoving)
      if (valid > 0) {
        g_Calibrations[selectedCalibration].computeCalibration()
        if (g_Calibrations[selectedCalibration].runstatus > 0) {
          g_Calibrations[selectedCalibration].saveCalibrationDataToFile()
          InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 1)
          this._results[this._myIndex].state = 1
        } else {
          this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
      }
      g_HECalibrationSettings = null
      tracer.addMessage('<- HEE: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  ACB: function (myIndex, cmdString) {
    tracer.addMessage('-> ACB init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this.copyRobotPose(7)
    var featureID = getFeatureIDsFromFeatureMask(this._featureMask)[0]

    tracer.addMessage('<- ACB init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> ACB: Tools done')
      InSightFunctions.fnSetCellValue('HECalibration.Selector', this._shuttlingPose - 1)
      InSightFunctions.fnSetCellValue('HECalibration.IsCameraMoving', this._isCameraMoving)
      InSightFunctions.fnSetCellValue('HECalibration.' + this._shuttlingPose + '.IsCameraMoving', this._isCameraMoving)

      g_AutoCalibRuntime = {}
      g_AutoCalibRuntime = g_Settings.AutoCalibration
      g_AutoCalibRuntime['innerDist'] = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Trainregion.InnerDist')
      g_AutoCalibRuntime['loopCnt'] = 0
      g_AutoCalibRuntime['stepCount'] = 0
      g_AutoCalibRuntime['direction'] = 1
      g_AutoCalibRuntime['minDist_Pixel'] = 55

      g_AutoCalibRuntime['rotAngle'] = (g_AutoCalibRuntime['AngleMax'] - g_AutoCalibRuntime['AngleMin']) / 10
      g_AutoCalibRuntime['advCalibPoints'] = []

      g_AutoCalibRuntime['lastMoveDistance_X'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM
      g_AutoCalibRuntime['lastMoveDistance_Y'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM
      g_AutoCalibRuntime['angleCompX'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM
      g_AutoCalibRuntime['angleCompY'] = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM

      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')
      this._results['1'].isValid = 1

      if (InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained') > 0) {
        if (targetValid > 0) {
          let firstPos = { 'X': this._robotPose.x, 'Y': this._robotPose.y, 'Z': this._robotPose.z, 'A': this._robotPose.thetaZ, 'B': this._robotPose.thetaY, 'C': this._robotPose.thetaX }
          g_AutoCalibRuntime['FirstPos'] = firstPos
          g_AutoCalibRuntime['PreCalibration'] = { 'Calibrated': 0 }
          g_AutoCalibRuntime['PreCalibPoses'] = []
          g_AutoCalibRuntime['CalibPoints'] = []
          g_AutoCalibRuntime['RobCalibPoses'] = []
          g_AutoCalibRuntime['NextRobotPose'] = {}
          g_AutoCalibRuntime['Compensation'] = {}
          g_AutoCalibRuntime['LastNextPos'] = { 'X': 0, 'Y': 0, 'Z': 0, 'A': 0, 'B': 0, 'C': 0 }

          g_Graphics['ShowCalibrationPoints_' + featureID] = 1

          g_Calibrations[this._shuttlingPose].deleteCalibrationDataFile()
          t.onReloadHeCalibrations()

          t.myCalibrations.calibrations[this._shuttlingPose] = new Calibration(this._shuttlingPose)
          t.myCalibrations.calibrations[this._shuttlingPose]['calibrationData']['isMoving'] = this._isCameraMoving

          var targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          var targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          var targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

          let nextRobotPose = t.myCalibrations.doAutoCalibration(this._robotPose.x, this._robotPose.y, this._robotPose.z, this._robotPose.thetaZ, this._robotPose.thetaY, this._robotPose.thetaX, targetX, targetY, targetAngle, targetValid)

          if (nextRobotPose.Valid) {
            nextRobotPose.NextX = Math.round(nextRobotPose.NextX * 10000) / 10000
            nextRobotPose.NextY = Math.round(nextRobotPose.NextY * 10000) / 10000
            nextRobotPose.NextAngle = Math.round(nextRobotPose.NextAngle * 10000) / 10000
          }
          if (this._results[this._myIndex].state >= 0) {
            this._results[this._myIndex].state = nextRobotPose.Valid
          }
          this._results[this._myIndex].data = nextRobotPose
        } else {
          this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_FOUND
          this._results[this._myIndex].data = []
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
        this._results[this._myIndex].data = []
      }

      tracer.addMessage('<- ACB: Tools done')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  AC: function (myIndex, cmdString) {
    tracer.addMessage('-> AC init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this.copyRobotPose(7)
    var featureID = getFeatureIDsFromFeatureMask(this._featureMask)[0]

    tracer.addMessage('<- AC init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> AC Tools done')

      var nextRobotPose = { 'Valid': 0 }
      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')

      if (InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained') > 0) {
        if (targetValid > 0) {
          let targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          let targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          let targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

          if (g_AutoCalibRuntime['PreCalibration'].Calibrated == 0) {
            nextRobotPose = t.myCalibrations.doAutoCalibration(this._robotPose.x, this._robotPose.y, this._robotPose.z, this._robotPose.thetaZ, this._robotPose.thetaY, this._robotPose.thetaX, targetX, targetY, targetAngle, targetValid)
          } else {
            g_Graphics['ShowCalibrationPoints_' + this._shuttlingPose] = 1
            let calib = t.myCalibrations.calibrations[this._shuttlingPose]

            calib.calibrationData.targetX.push(targetX)
            calib.calibrationData.targetY.push(targetY)
            calib.calibrationData.targetTheta.push(targetAngle)
            calib.calibrationData.targetValid.push(targetValid)
            calib.calibrationData.robotX.push(this._robotPose.x)
            calib.calibrationData.robotY.push(this._robotPose.y)
            calib.calibrationData.robotTheta.push(this._robotPose.thetaZ)

            calib.calibrationData.count = calib.calibrationData.targetX.length

            nextRobotPose = t.myCalibrations.doAutoCalibration(this._robotPose.x, this._robotPose.y, this._robotPose.z, this._robotPose.thetaZ, this._robotPose.thetaY, this._robotPose.thetaX, targetX, targetY, targetAngle, targetValid)

            if (nextRobotPose.Valid == 1) {
              let isMoving = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
              var valid = t.myCalibrations.CheckData(calib.calibrationData, isMoving)
              if (valid > 0) {
                calib.computeCalibration()
                if (calib.runstatus > 0) {
                  calib.saveCalibrationDataToFile()
                  InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 1)
                } else {

                }
              } else {
                console.log('Invalid calibration data! Calibration not saved!')
              }
            }
          }

          if (nextRobotPose.Valid) {
            nextRobotPose.NextX = Math.round(nextRobotPose.NextX * 10000) / 10000
            nextRobotPose.NextY = Math.round(nextRobotPose.NextY * 10000) / 10000
            nextRobotPose.NextAngle = Math.round(nextRobotPose.NextAngle * 10000) / 10000
          }

          this._results[this._myIndex].isValid = 1
          this._results[this._myIndex].state = nextRobotPose.Valid
          this._results[this._myIndex].data = nextRobotPose
        } else {
          this._results[this._myIndex].isValid = 1
          this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_FOUND
          this._results[this._myIndex].data = []
        }
      } else {
        this._results[this._myIndex].isValid = 1
        this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
        this._results[this._myIndex].data = []
      }
      tracer.addMessage('<- AC Tools done')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  CCB: function (myIndex, cmdString) {
    tracer.addMessage('-> CCB init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)

    var swapHandedness = parseInt(this._splittedCmd[6])
    var featureOffsetX = parseFloat(this._splittedCmd[7])
    var featureOffsetY = parseFloat(this._splittedCmd[8])

    tracer.addMessage('<- CCB init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> CCB: Execute')
      InSightFunctions.fnSetCellValue('HECalibration.IsCameraMoving', this._isCameraMoving)
      InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 0)
      g_CustomCalibrationSettings = new CustomCalibrationSettings(swapHandedness, featureOffsetX, featureOffsetY, -1)

      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1

      tracer.addMessage('<- CCB: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  CC: function (myIndex, cmdString) {
    tracer.addMessage('-> CC init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)

    if (g_CustomCalibrationSettings == null) {
      this._validCMD = ECodes.E_NO_START_COMMAND
      return
    }

    this.copyRobotPose(7)
    var featureID = getFeatureIDsFromFeatureMask(this._featureMask)[0]
    if (featureID == 0) {
      featureID = 1
    }
    this.execute = function (t) {
        tracer.addMessage('-> Execute ' + timeTracker.getElapsedTime())
        this._enabledFeatures = 0

        setFeatureAcqSettings(this._exposureSet - 1)

        if (this._logImageType.length > 2) {
            InSightFunctions.fnSetCellValue(this._logImageType, 1)
        }

        if (t.triggerMode == 32) {
            InSightFunctions.fnSetEvent(32)
        }

        if (g_CustomCalibrationSettings.calibrationID < 1) {
            g_CustomCalibrationSettings.calibrationID = this._shuttlingPose
            g_Calibrations[this._shuttlingPose].deleteCalibrationDataFile()
            t.onReloadHeCalibrations()
            g_Calibrations[this._shuttlingPose] = new Calibration(this._shuttlingPose)
          g_Calibrations[this._shuttlingPose]['calibrationData']['isMoving'] = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
          InSightFunctions.fnSetCellValue('HECalibration.' + this._shuttlingPose + '.IsCameraMoving', g_Calibrations[this._shuttlingPose]['calibrationData']['isMoving'])
        }
        tracer.addMessage('<- CC init ' + timeTracker.getElapsedTime())
        tracer.addMessage('<- Execute ' + timeTracker.getElapsedTime())
        return States.WAITING_FOR_IMAGE_ACQUIRED
    }
    this.toolsDone = function (t) {
      tracer.addMessage('-> CC: Tools done')
      g_Graphics['ShowCalibrationPoints_' + this._shuttlingPose] = 1
      let targetValid = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Valid')
      let targetTrained = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern.Trained')
      this._results[this._myIndex].isValid = 1

      if (targetTrained > 0) {
        if (targetValid > 0) {
          var targetX = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_X')
          var targetY = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Y')
          var targetAngle = InSightFunctions.fnGetCellValue('Target.' + featureID + '.Pattern_Angle')

          g_Calibrations[this._shuttlingPose].calibrationData.targetX.push(targetX)
          g_Calibrations[this._shuttlingPose].calibrationData.targetY.push(targetY)
          g_Calibrations[this._shuttlingPose].calibrationData.targetTheta.push(targetAngle)
          g_Calibrations[this._shuttlingPose].calibrationData.targetValid.push(targetValid)
          g_Calibrations[this._shuttlingPose].calibrationData.robotX.push(this._robotPose.x)
          g_Calibrations[this._shuttlingPose].calibrationData.robotY.push(this._robotPose.y)
          g_Calibrations[this._shuttlingPose].calibrationData.robotTheta.push(this._robotPose.thetaZ)

          g_Calibrations[this._shuttlingPose].calibrationData.count = g_Calibrations[this._shuttlingPose].calibrationData.targetX.length
          this._results[this._myIndex].state = 1
        } else {
          this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_FOUND
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_FEATURE_NOT_TRAINED
      }

      tracer.addMessage('<- CC: Tools done')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  CCE: function (myIndex, cmdString) {
    tracer.addMessage('-> CCE init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)

    if (g_CustomCalibrationSettings == null) {
      this._validCMD = ECodes.E_NO_START_COMMAND
      return
    }

    var selectedCalibration = g_CustomCalibrationSettings.calibrationID
    tracer.addMessage('selectedCalibration ' + selectedCalibration)

    tracer.addMessage('<- CCE init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> CCE: Execute')
      g_Graphics['ShowCalibrationPoints_' + selectedCalibration] = 1
      this._results[this._myIndex].isValid = 1
      let isMoving = InSightFunctions.fnGetCellValue('HECalibration.IsCameraMoving')
      var ccData = t.myCalibrations.doCustomCalibration(g_Calibrations[selectedCalibration].calibrationData,
        g_CustomCalibrationSettings.swapHandedness, g_CustomCalibrationSettings.featureOffsetX, g_CustomCalibrationSettings.featureOffsetY)

      if (ccData != 0) {
        var valid = t.myCalibrations.CheckData(g_Calibrations[selectedCalibration].calibrationData, isMoving)

        if (valid > 0) {
          g_Calibrations[selectedCalibration].computeCalibration()
          if (g_Calibrations[selectedCalibration].runstatus > 0) {
            g_Calibrations[selectedCalibration].saveCalibrationDataToFile()
            InSightFunctions.fnSetCellValue('HECalibration.NewCalibrationDone', 1)
            this._results[this._myIndex].state = 1
          } else {
            this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
          }
        } else {
          this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
        }
      } else {
        this._results[this._myIndex].state = ECodes.E_CALIBRATION_FAILED
      }

      g_CustomCalibrationSettings = null
      tracer.addMessage('<- CCE: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  TA: function (myIndex, cmdString) {
    tracer.addMessage('-> TA init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsTrainImage'
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
    }

    tracer.addMessage('<- TA init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> TA: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          g_TrainedFeatures[f][this._gripperID1] = new Feature(0, 0, 0, 0)
          g_TrainedFeatures[f][this._gripperID1].valid = transformed.valid
          g_TrainedFeatures[f][this._gripperID1].x = transformed.x
          g_TrainedFeatures[f][this._gripperID1].y = transformed.y
          g_TrainedFeatures[f][this._gripperID1].thetaInDegrees = transformed.thetaInDegrees

          this._results[this._myIndex].data.push(transformed)
        }
      }

      InSightFunctions.fnSetEvent(83)

      tracer.addMessage('<- TA: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  XA: function (myIndex, cmdString) {
    tracer.addMessage('-> XA init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
    }
    tracer.addMessage('<- XA init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> XA: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      g_Graphics.ShowCrossHair = []

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push(transformed)

          if (g_TrainedFeatures[f][this._gripperID1].valid > 0) {
            let imgPos = []
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[f][this._gripperID1].x, g_TrainedFeatures[f][this._gripperID1].y, g_TrainedFeatures[f][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }

      tracer.addMessage('<- XA: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }
  },
  XAS: function (myIndex, cmdString) {
    tracer.addMessage('-> XAS init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
    }

    tracer.addMessage('<- XAS init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> XAS: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      g_Graphics.ShowCrossHair = []

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        let score = -1

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
            score = InSightFunctions.fnGetCellValue('Target.' + f + '.Pattern_Score')
            score = (Math.round(score*1000)/1000)
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push([transformed, score])

          if (g_TrainedFeatures[f][this._gripperID1].valid > 0) {
            let imgPos = []
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[f][this._gripperID1].x, g_TrainedFeatures[f][this._gripperID1].y, g_TrainedFeatures[f][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }

      tracer.addMessage('<- XAS: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }
  },
  XALC: function (myIndex, cmdString) {
    tracer.addMessage('-> XALC init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
    }
    tracer.addMessage('<- XALC init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> XALC: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      g_Graphics.ShowCrossHair = []

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push(transformed)

          if (g_TrainedFeatures[f][this._gripperID1].valid > 0) {
            let imgPos = []
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[f][this._gripperID1].x, g_TrainedFeatures[f][this._gripperID1].y, g_TrainedFeatures[f][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }

      tracer.addMessage('<- XALC: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }
  },
  XAMP: function (myIndex, cmdString) {
    tracer.addMessage('-> XAMP init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
    }
    tracer.addMessage('<- XAMP init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> XAMP: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      g_Graphics.ShowCrossHair = []

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push(transformed)

          if (g_TrainedFeatures[f][this._gripperID1].valid > 0) {
            let imgPos = []
            imgPos = getImageFromWorld(g_Calibrations, this._shuttlingPose, g_TrainedFeatures[f][this._gripperID1].x, g_TrainedFeatures[f][this._gripperID1].y, g_TrainedFeatures[f][this._gripperID1].thetaInDegrees, 0, 0, 0)
            g_Graphics.ShowCrossHair.push([imgPos.x, imgPos.y, imgPos.thetaInDegrees, imgPos.valid])
          }
        }
      }

      tracer.addMessage('<- XAMP: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_SLAVE_RESULT
    }
  },

  XI: function (myIndex, cmdString) {
    tracer.addMessage('-> XI init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    var inspectionID = parseInt(this._splittedCmd[6])
    tracer.addMessage('<- XI init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> XI: Execute')
      this._enabledFeatures = 0
      if (g_Inspections.hasOwnProperty(inspectionID)) {
        setInspectionAcqSettings(inspectionID, 1)        

        if (this._logImageType.length > 2) {
          InSightFunctions.fnSetCellValue(this._logImageType, 1)
        }

        if (t.triggerMode == 32) {
          InSightFunctions.fnSetEvent(32)
        }
      } else {
        this._results[this._myIndex].isValid = 1
        this._results[this._myIndex].state = ECodes.E_INVALID_ARGUMENT
        this._results[this._myIndex].data.push({})
        this._results.error = ECodes.E_INVALID_ARGUMENT
      }

      tracer.addMessage('<- XI: Execute')
      return States.WAITING_FOR_IMAGE_ACQUIRED
    }

    this.imgAcquired = function () {
      tracer.addMessage('-> XI: Image acquired ' + timeTracker.getElapsedTime())
      this._featureMask = inspectionID << 8
      this._enabledFeatures = this._featureMask

      tracer.addMessage('<- XI: Image acquired ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_TOOLS_DONE
    }

    this.toolsDone = function (t) {
      tracer.addMessage('-> XI: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      tracer.addMessage('<- XI: Tools done ' + timeTracker.getElapsedTime())
      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  SGP: function (myIndex, cmdString) {
    tracer.addMessage('-> SGP init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    var mode = this._splittedCmd[6]
    var newGoldenPose = new Feature(0, 0, 0, 0)
    let featureID = this._featureMask
    
    newGoldenPose.x = parseFloat(this._splittedCmd[7])
    newGoldenPose.y = parseFloat(this._splittedCmd[8])
    newGoldenPose.thetaInDegrees = parseFloat(this._splittedCmd[9])

    newGoldenPose.valid = 1

    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
    }

    tracer.addMessage('<- SGP init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> SGP: Execute')

      this._results[this._myIndex].state = ECodes.E_UNSPECIFIED
      this._results[this._myIndex].isValid = 1

      if (false) {
        this._results[this._myIndex].state = ECodes.E_COMBINATION_NOT_ALLOWED
      } else {
        if ((mode == CoordinateSystem.HOME2D) || (mode == CoordinateSystem[CoordinateSystem.HOME2D])) {
          this._results[this._myIndex].state = 1
        } else if ((mode == CoordinateSystem.CAM2D) || (mode == CoordinateSystem[CoordinateSystem.CAM2D])) {
          let newGPInWorld = getWorldFromCam(g_Calibrations, this._shuttlingPose, newGoldenPose.x, newGoldenPose.y, newGoldenPose.thetaInDegrees, 0, 0, 0)
          newGoldenPose.valid = newGPInWorld.valid
          if (newGoldenPose.valid > 0) {
            newGoldenPose.x = newGPInWorld.x
            newGoldenPose.y = newGPInWorld.y
            newGoldenPose.thetaInDegrees = newGPInWorld.thetaInDegrees
          }

          this._results[this._myIndex].state = newGoldenPose.valid
        } else if ((mode == CoordinateSystem.RAW2D) || (mode == CoordinateSystem[CoordinateSystem.RAW2D])) {
          let newGPInWorld = getWorldFromImage(g_Calibrations, this._shuttlingPose, newGoldenPose.x, newGoldenPose.y, newGoldenPose.thetaInDegrees, 0, 0, 0)
          newGoldenPose.valid = newGPInWorld.valid
          if (newGoldenPose.valid > 0) {
            newGoldenPose.x = newGPInWorld.x
            newGoldenPose.y = newGPInWorld.y
            newGoldenPose.thetaInDegrees = newGPInWorld.thetaInDegrees
          }
          this._results[this._myIndex].state = newGoldenPose.valid
        }
      }

      if (newGoldenPose.valid > 0) {
        if (!g_TrainedFeatures.hasOwnProperty(featureID)) {
          g_TrainedFeatures[featureID] = []
          for (let g = 0; g < MAX_GRIPPERS; g++) {
            g_TrainedFeatures[featureID][g] = new Feature(0, 0, 0, 0)
          }
        }

        g_TrainedFeatures[featureID][this._gripperID1].x = newGoldenPose.x
        g_TrainedFeatures[featureID][this._gripperID1].y = newGoldenPose.y
        g_TrainedFeatures[featureID][this._gripperID1].thetaInDegrees = newGoldenPose.thetaInDegrees
        g_TrainedFeatures[featureID][this._gripperID1].valid = newGoldenPose.valid

        this._results[this._myIndex].data.push(newGoldenPose)

        InSightFunctions.fnSetEvent(83)
      }

      tracer.addMessage('<- SGP: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  GGP: function (myIndex, cmdString) {
    tracer.addMessage('-> GGP init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    if (this._isCameraMoving) {
      this._validCMD = ECodes.E_NOT_SUPPORTED
    }
    tracer.addMessage('<- GGP init ' + timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> GGP: Execute')
      var mode = this._splittedCmd[6]
      let featureID = this._featureMask
      this._results[this._myIndex].state = ECodes.E_UNSPECIFIED
      this._results[this._myIndex].isValid = 1

      if (g_TrainedFeatures[featureID][this._gripperID1].valid > 0) {
        let goldenPose = cloneObj(g_TrainedFeatures[featureID][this._gripperID1])
        if ((mode == CoordinateSystem.HOME2D) || (mode == CoordinateSystem[CoordinateSystem.HOME2D])) {
    
        } else if ((mode == CoordinateSystem.CAM2D) || (mode == CoordinateSystem[CoordinateSystem.CAM2D])) {
          let goldenPoseInCam2D = getCamFromWorld(g_Calibrations, this._shuttlingPose, goldenPose.x, goldenPose.y, goldenPose.thetaInDegrees)
          goldenPose.valid = goldenPoseInCam2D.valid
          if (goldenPose.valid > 0) {
            goldenPose.x = goldenPoseInCam2D.x
            goldenPose.y = goldenPoseInCam2D.y
            goldenPose.thetaInDegrees = goldenPoseInCam2D.thetaInDegrees
          }
        } else if ((mode == CoordinateSystem.RAW2D) || (mode == CoordinateSystem[CoordinateSystem.RAW2D])) {
          let goldenPoseInRaw2D = getImageFromWorld(g_Calibrations, this._shuttlingPose, goldenPose.x, goldenPose.y, goldenPose.thetaInDegrees, 0, 0, 0)
          goldenPose.valid = goldenPoseInRaw2D.valid
          if (goldenPose.valid > 0) {
            goldenPose.x = goldenPoseInRaw2D.x
            goldenPose.y = goldenPoseInRaw2D.y
            goldenPose.thetaInDegrees = goldenPoseInRaw2D.thetaInDegrees
          }
        }

        this._results[this._myIndex].state = goldenPose.valid
        this._results[this._myIndex].data.push(goldenPose)
      } else {
        this._results[this._myIndex].state = ECodes.E_TARGET_POSE_NOT_TRAINED
      }
      tracer.addMessage('<- GGP: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  GCP: function (myIndex, cmdString) {
    tracer.addMessage('-> GCP init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    tracer.addMessage('<- GCP init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> GCP: Tools done ' + timeTracker.getElapsedTime())
      var mode = this._splittedCmd[6]
      this._results[this._myIndex]['isValid'] = 1
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }

      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let home2D = new Feature(0, 0, 0, 0)
        let transformed = new Feature(0, 0, 0, 0, 0, 0)

        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            home2D = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)

            if (this._results[this._myIndex].state >= 0) {

              if ((mode == CoordinateSystem.HOME2D) || (mode == CoordinateSystem[CoordinateSystem.HOME2D])) {
                transformed.valid = home2D.valid
                if (transformed.valid > 0) {
                  transformed.x = home2D.x
                  transformed.y = home2D.y
                  transformed.thetaInDegrees = home2D.thetaInDegrees
                }

              } else if ((mode == CoordinateSystem.CAM2D) || (mode == CoordinateSystem[CoordinateSystem.CAM2D])) {
                transformed = getCamFromWorld(g_Calibrations, this._shuttlingPose, home2D.x, home2D.y, home2D.thetaInDegrees)
              } else if ((mode == CoordinateSystem.RAW2D) || (mode == CoordinateSystem[CoordinateSystem.RAW2D])) {
                transformed = getImageFromWorld(g_Calibrations, this._shuttlingPose, home2D.x, home2D.y, home2D.thetaInDegrees, 0, 0, 0)
              }
            }
            this._results[this._myIndex].state = transformed.valid
            this._results[this._myIndex].data.push(transformed)


          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }

          this._results[this._myIndex].data.push(transformed)
        }
      }

      tracer.addMessage('<- GCP: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  TT: function (myIndex, cmdString) {
    tracer.addMessage('-> TT init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsTrainImage'
    this.copyRobotPose(6)

    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }

    tracer.addMessage('<- TT init ' + timeTracker.getElapsedTime())
  },
  TTR: function (myIndex, cmdString) { SlaveBase.call(this, myIndex, cmdString) },
  XT: function (myIndex, cmdString) {
    tracer.addMessage('-> XT init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)

    if (g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration.isCameraMoving_
    }
    if (((this._featureMask & MASK_ID_2) >> SHIFT_ID_2) > 0) {
      this._computeBothParts = 1
    }

    tracer.addMessage('<- XT init ' + timeTracker.getElapsedTime())

    this.imgAcquired = function () {
      tracer.addMessage('-> XT: Image acquired ' + timeTracker.getElapsedTime())
      let features = (this._featureMask & MASK_ID_1)

      if (this._computeBothParts == 1) {

        features = features | ((this._featureMask & MASK_ID_2) >> SHIFT_ID_2)
      }

      this._enabledFeatures = features
      tracer.addMessage('<- XT: Image acquired ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_TOOLS_DONE
    }

    this.toolsDone = function (t) {
      tracer.addMessage('-> XT: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      let transformed = new Feature(0, 0, 0, 0)
      let transformed2 = new Feature(0, 0, 0, 0)

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

        if (this.isFeatureEnabled(f) == true) {

          if (g_CurrentFeatures[f].valid > 0) {

            if ((this._featureMask & MASK_ID_1) & f) {
              transformed = new Feature(0, 0, 0, 0)
              let shuttlingPose = this._shuttlingPose & MASK_ID_1
              let isCameraMoving = this._isCameraMoving & MASK_ID_1
              let isPartMoving = this._isPartMoving & MASK_ID_1

              transformed = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
              this._results[this._myIndex].data.push(transformed)
              if (this._results[this._myIndex].state >= 0) {
                this._results[this._myIndex].state = transformed.valid
              }
            }

            if (this._computeBothParts == 1) {
              if (((this._featureMask & MASK_ID_2) >> SHIFT_ID_2) & f) {
                transformed2 = new Feature(0, 0, 0, 0)
                let shuttlingPose = (this._shuttlingPose & MASK_ID_2) >> SHIFT_ID_2
                let isCameraMoving = (this._isCameraMoving & MASK_ID_2) >> SHIFT_ID_2
                let isPartMoving = (this._isPartMoving & MASK_ID_2) >> SHIFT_ID_2

                transformed2 = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
                this._results[this._myIndex].data.push(transformed2)
                if (this._results[this._myIndex].state >= 0) {
                  this._results[this._myIndex].state = transformed2.valid
                }
              }
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
        }
      }

      tracer.addMessage('<- XT: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  XTS: function (myIndex, cmdString) {
    tracer.addMessage('-> XTS init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    tracer.addMessage('<- XTS init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> XTS: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
        let transformed = new Feature(0, 0, 0, 0)
        let score = -1
        if (this.isFeatureEnabled(f) == true) {
          if (g_CurrentFeatures[f].valid > 0) {
            transformed = getTransformed(g_Calibrations, this._shuttlingPose, this._isCameraMoving, this._isPartMoving, g_CurrentFeatures[f], this._robotPose)
            if (this._results[this._myIndex].state >= 0) {
              this._results[this._myIndex].state = transformed.valid
            }
            score = InSightFunctions.fnGetCellValue('Target.' + f + '.Pattern_Score') 
            score = (Math.round(score * 1000) / 1000)
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
          this._results[this._myIndex].data.push([transformed, score])
        }
      }

      tracer.addMessage('<- XTS: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  XTLC: function (myIndex, cmdString) {
    tracer.addMessage('-> XTLC init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)

    if (g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration.isCameraMoving_
    }
    if (((this._featureMask & MASK_ID_2) >> SHIFT_ID_2) > 0) {
      this._computeBothParts = 1
    }

    tracer.addMessage('<- XTLC init ' + timeTracker.getElapsedTime())

    this.imgAcquired = function () {
      tracer.addMessage('-> XTLC: Image acquired ' + timeTracker.getElapsedTime())
      let features = (this._featureMask & MASK_ID_1)

      if (this._computeBothParts == 1) {

        features = features | ((this._featureMask & MASK_ID_2) >> SHIFT_ID_2)
      }

      this._enabledFeatures = features
      tracer.addMessage('<- XTLC: Image acquired ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_TOOLS_DONE
    }

    this.toolsDone = function (t) {
      tracer.addMessage('-> XTLC: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      let transformed = new Feature(0, 0, 0, 0)
      let transformed2 = new Feature(0, 0, 0, 0)

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

        if (this.isFeatureEnabled(f) == true) {

          if (g_CurrentFeatures[f].valid > 0) {

            if ((this._featureMask & MASK_ID_1) & f) {
              transformed = new Feature(0, 0, 0, 0)
              let shuttlingPose = this._shuttlingPose & MASK_ID_1
              let isCameraMoving = this._isCameraMoving & MASK_ID_1
              let isPartMoving = this._isPartMoving & MASK_ID_1

              transformed = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
              this._results[this._myIndex].data.push(transformed)
              if (this._results[this._myIndex].state >= 0) {
                this._results[this._myIndex].state = transformed.valid
              }
            }

            if (this._computeBothParts == 1) {
              if (((this._featureMask & MASK_ID_2) >> SHIFT_ID_2) & f) {
                transformed2 = new Feature(0, 0, 0, 0)
                let shuttlingPose = (this._shuttlingPose & MASK_ID_2) >> SHIFT_ID_2
                let isCameraMoving = (this._isCameraMoving & MASK_ID_2) >> SHIFT_ID_2
                let isPartMoving = (this._isPartMoving & MASK_ID_2) >> SHIFT_ID_2

                transformed2 = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
                this._results[this._myIndex].data.push(transformed2)
                if (this._results[this._myIndex].state >= 0) {
                  this._results[this._myIndex].state = transformed2.valid
                }
              }
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
        }
      }

      tracer.addMessage('<- XTLC: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  XTMP: function (myIndex, cmdString) {
    tracer.addMessage('-> XTMP init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)

    if (g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration.isCameraMoving_
    }
    

    tracer.addMessage('<- XTMP init ' + timeTracker.getElapsedTime())

    this.toolsDone = function (t) {
      tracer.addMessage('-> XTMP: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      let transformed = new Feature(0, 0, 0, 0)

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

        if (this.isFeatureEnabled(f) == true) {

          if (g_CurrentFeatures[f].valid > 0) {

            if ((this._featureMask & MASK_ID_1) & f) {
              transformed = new Feature(0, 0, 0, 0)
              let shuttlingPose = this._shuttlingPose & MASK_ID_1
              let isCameraMoving = this._isCameraMoving & MASK_ID_1
              let isPartMoving = this._isPartMoving & MASK_ID_1

              transformed = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
              this._results[this._myIndex].data.push(transformed)
              if (this._results[this._myIndex].state >= 0) {
                this._results[this._myIndex].state = transformed.valid
              }
            }
          } else {
            this._results[this._myIndex].state = g_CurrentFeatures[f].valid
          }
        }
      }

      tracer.addMessage('<- XTMP: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_NEW_COMMAND
    }
  },
  XT2: function (myIndex, cmdString) {
    tracer.addMessage('-> XT2 init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)

    if (g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose & MASK_ID_1].calibration.isCameraMoving_
    }
    if (((this._featureMask & MASK_ID_2) >> SHIFT_ID_2) > 0) {
      this._computeBothParts = 1
    }

    tracer.addMessage('<- XT2 init ' + timeTracker.getElapsedTime())

    this.imgAcquired = function () {
      tracer.addMessage('-> XT2: Image acquired ' + timeTracker.getElapsedTime())
      let features = (this._featureMask & MASK_ID_1)

      if (this._computeBothParts == 1) {

        features = features | ((this._featureMask & MASK_ID_2) >> SHIFT_ID_2)
      }

      this._enabledFeatures = features
      tracer.addMessage('<- XT2: Image acquired ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_TOOLS_DONE
    }

    this.toolsDone = function (t) {
      tracer.addMessage('-> XT2: Tools done ' + timeTracker.getElapsedTime())
      if (this._logImageType.length > 2) {
        InSightFunctions.fnSetCellValue(this._logImageType, 0)
      }
      let transformed = new Feature(0, 0, 0, 0)
      let transformed2 = new Feature(0, 0, 0, 0)

      this._results[this._myIndex]['isValid'] = 1
      for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

        if (this.isFeatureEnabled(f) == true) {

          if (g_CurrentFeatures[f].valid > 0) {

            if ((this._featureMask & MASK_ID_1) & f) {
              transformed = new Feature(0, 0, 0, 0)
              let shuttlingPose = this._shuttlingPose & MASK_ID_1
              let isCameraMoving = this._isCameraMoving & MASK_ID_1
              let isPartMoving = this._isPartMoving & MASK_ID_1

              transformed = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
              this._results[this._myIndex].data.push(transformed)
              if (this._results[this._myIndex].state >= 0) {
                this._results[this._myIndex].state = transformed.valid
              }
            }

            if (this._computeBothParts == 1) {
              if (((this._featureMask & MASK_ID_2) >> SHIFT_ID_2) & f) {
                transformed2 = new Feature(0, 0, 0, 0)
                let shuttlingPose = (this._shuttlingPose & MASK_ID_2) >> SHIFT_ID_2
                let isCameraMoving = (this._isCameraMoving & MASK_ID_2) >> SHIFT_ID_2
                let isPartMoving = (this._isPartMoving & MASK_ID_2) >> SHIFT_ID_2

                transformed2 = getTransformed(g_Calibrations, shuttlingPose, isCameraMoving, isPartMoving, g_CurrentFeatures[f], this._robotPose)
                this._results[this._myIndex].data.push(transformed2)
                if (this._results[this._myIndex].state >= 0) {
                  this._results[this._myIndex].state = transformed2.valid
                }
              }
            }
          } else {
            let emptyData = new Feature(0, 0, 0, g_CurrentFeatures[f].valid)
            this._results[this._myIndex].data.push(emptyData)            
          }
        }
      }

      tracer.addMessage('<- XT2: Tools done ' + timeTracker.getElapsedTime())

      return States.WAITING_FOR_NEW_COMMAND
    }
  },

  CP: function (myIndex, cmdString) { SlaveBase.call(this, myIndex, cmdString) },
  LF: function (myIndex, cmdString) {
    tracer.addMessage('-> LF init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    this._logImageType = 'LogImage.IsProductionImage'
    this.copyRobotPose(7)
    if (g_Calibrations[this._shuttlingPose].calibration !== null) {
      this._isCameraMoving = g_Calibrations[this._shuttlingPose].calibration.isCameraMoving_
    }
    tracer.addMessage('<- LF init ' + timeTracker.getElapsedTime())
  },
  TP: function (myIndex, cmdString) { SlaveBase.call(this, myIndex, cmdString) },
  TPR: function (myIndex, cmdString) { SlaveBase.call(this, myIndex, cmdString) },
  GP: function (myIndex, cmdString) { SlaveBase.call(this, myIndex, cmdString) },
  PID: function (myIndex, cmdString) {
    tracer.addMessage('-> PID init ' + timeTracker.getElapsedTime())
    SlaveBase.call(this, myIndex, cmdString)
    tracer.addMessage('<- PID init ' + +timeTracker.getElapsedTime())

    this.execute = function (t) {
      tracer.addMessage('-> PID: Execute')

      writeCellValue("LogImage.ProductID", this._splittedCmd[6])
      
      this._results[this._myIndex].data.push({})
      this._results[this._myIndex].isValid = 1
      this._results[this._myIndex].state = 1
      tracer.addMessage('<- PID: Execute')
      return States.WAITING_FOR_NEW_COMMAND
    }
  }
}
//* ***************************************************************************/
//* ***************************************************************************/
//* ***************************************************************************/

//* ***************************************************************************/
// InSight functions
//* ***************************************************************************/
var InSightFunctions = {

  fnGetCellValue: new tools.GetCellValue(),
  fnSetCellValue: new tools.SetCellValue(),
  fnGetCellName: new tools.GetCellName(),
  fnSetEvent: new tools.SetEvent(),
  fnUpdateGui: new tools.UpdateGui(),
  fnStringf: new tools.Stringf(),
  fnLeft: new tools.Left(),
  fnLineToLine: new tools.LineToLine(),
  fnNow: new tools.Now(1),
  fnGetGlock: new tools.GetClock(),

  fnGetSystemConfig: new tools.GetSystemConfig(),
  fnCalibrateAdvanced: new tools.CalibrateAdvanced(),
  fnTransPixelToWorld: new tools.TransPixelToWorld(),
  fnTransWorldToPixel: new tools.TransWorldToPixel(),
  fnPoint: new tools.Point(),
  fnPlotPoint: new tools.PlotPoint(),
  fnPointToPoint: new tools.PointToPoint()
}

//* ***************************************************************************/
// My Details
//* ***************************************************************************/
/**
 * Holds all informations about the camera to have 
 * the "important" things on one place 
 * */
function MyDetails() {
  this.myIP = InSightFunctions.fnGetSystemConfig('hostip')
  this.myName = InSightFunctions.fnGetSystemConfig('hostname')
  this.myMAC = InSightFunctions.fnStringf('%M')
    this.myIndex = 0
  let services = InSightFunctions.fnGetSystemConfig('ServicesEnabled')
  this.slmp = 0
  this.etherNetIP = 0
  if (services == 16) {
    this.slmp = 1
  } else if (services == 2){
    this.etherNetIP = 1
  }
  this.iAmMaster = false

  this.fullJobName = InSightFunctions.fnGetSystemConfig('jobname')

  this.jobName = this.getJobName(this.fullJobName)
};
/**
 * Writes the informations to the spread sheet
 * */
MyDetails.prototype.writeToSheet = function () {
  
  writeCellValue('Internal.Sensor.MyName', this.myName)
  writeCellValue('Internal.Sensor.MyIP', this.myIP)
  writeCellValue('Internal.Sensor.MyMAC', this.myMAC)

  writeCellValue('Internal.Sensor.IamMaster', this.iAmMaster)
  writeCellValue('Internal.Sensor.MyIndex', this.myIndex)
  writeCellValue('Internal.Sensor.SLMP', this.slmp)
  writeCellValue('Internal.Sensor.EtherNetIP', this.etherNetIP)

  writeCellValue('Internal.Recipe.FullJobName', this.fullJobName)
  writeCellValue('Internal.Recipe.JobName', this.jobName)  
}
/**
 * Returns the job name only (without path)
 * @param {any} fullName the full job name (with path if a FTP server is used)
 */
MyDetails.prototype.getJobName = function (fullName) {
  var splittedName = []
  splittedName = fullName.split('\\')

  if (splittedName[splittedName.length - 1].indexOf('/') !== -1) {
    splittedName = splittedName[splittedName.length - 1].split('/')
  }

  var name = splittedName[splittedName.length - 1].replace('.job', '')
  return name
}
/**
 * Holds all informations about the sensor
 * */
function Sensor() {
  this.version = -1
  this.name = ''
  this.description = ''
  this.plcPort = 7890
  this.useShuttledCameras = 0
  this.shuttledCameraCalibrations = {}
  this.cams = {}
  this.jobserver = {}
  this.logImageFtp = {}
  this.logDataFtp = {}
  this.messageLengthHeader = false
};
/**
 * Initialize the sensor object with the informations from the sensor configuration file
 * @param {any} sensorConfig
 */
Sensor.prototype.initFromSensorConfig = function (sensorConfig) {
  this.version = sensorConfig.sensor.Version
  this.name = sensorConfig.sensor.Name
  this.description = sensorConfig.sensor.Description
  this.plcPort = sensorConfig.sensor.PLCPort
  this.useShuttledCameras = sensorConfig.sensor.UseShuttledCameras
  this.shuttledCameraCalibrations = sensorConfig.sensor.ShuttledCameraCalibrations
  this.cams = sensorConfig.sensor.Cams
  this.jobserver = sensorConfig.sensor.Jobserver
  this.logImageFtp = sensorConfig.sensor.LogImageFtp
  this.messageLengthHeader = !!sensorConfig.sensor.MessageLengthHeader

  if (sensorConfig.sensor.hasOwnProperty('LogDataFtp')) {
    this.logDataFtp = sensorConfig.sensor.LogDataFtp
  }
  else {
    this.logDataFtp = {
      "Enabled": false,
      "Hostname": "0.0.0.0",
      "UserName": "",
      "Password": "",
      "Port": 21
    }
  }
}
/**
 * Writes the informations to the spread sheet
 * */
Sensor.prototype.writeToSheet = function () {
  
  writeCellValue('Internal.Sensor.Description', this.description)

  writeCellValue('Internal.Sensor.PLCPort', this.plcPort)
  writeCellValue('Communication.MessageLengthHeader', this.messageLengthHeader)
  writeCellValue('Internal.Sensor.UseShuttledCameras', this.useShuttledCameras)

  for (var i = 1; i <= MAX_CAMERAS; i++) {
    let tagName = ''
    let ip = ''
    let mac = '---'
    let master = 0

    if (this.cams.hasOwnProperty('Cam_' + i.toString()) === true) {
      var cam = this.cams['Cam_' + i.toString()]
      ip = cam.IPAddress

      mac = cam.MACAddress
      master = cam.Master
    }
    tagName = 'Internal.Sensor.IP_' + i.toString()
    writeCellValue(tagName, ip)
    tagName = 'Internal.Sensor.MAC_' + i.toString()
    writeCellValue(tagName, mac)
    tagName = 'Internal.Sensor.Master_' + i.toString()
    writeCellValue(tagName, master)
  }
}
/**
 * Returns the camera ID 
 * The IP address is used for the search
 * @param {any} ip IP to search for
 */
Sensor.prototype.findIndexByIp = function (ip) {
  let ret = -1
  for (var i in this.cams) {
    if (this.cams[i]['IPAddress'] == ip) {
      ret = parseInt(i.replace('Cam_', ''))
    }
  }
  return ret
}

function Recipes(sensorConfiguration) {
  this.recipes = {}
  let recipeKeys = Object.keys(sensorConfiguration.recipes)
  for (let index in recipeKeys) {
    this.recipes[recipeKeys[index].toLowerCase()] = sensorConfiguration.recipes[recipeKeys[index]]
  }
};

//* ***************************************************************************/
// SensorConfiguration
//* ***************************************************************************/

function SensorConfiguration() {
  this.sensor = null
  this.recipes = null
  this.configuration = null
};

SensorConfiguration.prototype.loadFromFile = function () {
  let fileObj = cogUtils.loadFile(SENSORCONFIG_FILENAME)
  let ret = false

  if (fileObj != 'undefined') {
    this.init(fileObj)
    ret = true
  }
  return ret
}

SensorConfiguration.prototype.init = function (sensorConfiguration) {
  this.configuration = sensorConfiguration
  this.sensor = sensorConfiguration.Sensor
  this.recipes = sensorConfiguration.Recipes
}

//* ***************************************************************************/
// CameraConfiguration
//* ***************************************************************************/

function LoadCameraConfigFile() {
  let cc = cogUtils.loadFile(CAMERACONFIG_FILENAME)

  if (cc != 'undefined') {
    g_Settings.LogImageFtp = cc['LogImageFtp']
    g_Settings.AutoCalibration = cc['AutoCalibration']
    g_Settings.CustomCalibration = cc['CustomCalibration']
    g_Settings.SensorName = cc['SensorName']
    
    if (cc.hasOwnProperty('IsRobotMounted')) {
      g_Settings.IsRobotMounted = cc['IsRobotMounted']
    } else {
      g_Settings.IsRobotMounted = false
    }

    if (cc.hasOwnProperty('RunUncalibrated')) {
      g_Settings.RunUncalibrated = cc['RunUncalibrated']
    } else {
      g_Settings.RunUncalibrated = false
    }

    if (cc.hasOwnProperty('LogDataFtp')) {
      g_Settings.LogDataFtp =cc.LogDataFtp
    }
    else {
      g_Settings.LogDataFtp = {
        "Enabled": false,
        "Hostname": "0.0.0.0",
        "UserName": "",
        "Password": "",
        "Port": 21
      }
    }

    if (!cc.AutoCalibration.hasOwnProperty('FirstStepSize')) {      
      g_Settings.AutoCalibration.FirstStepSize = START_MOVE_DISTANCE_MM
    } 
  }
  
  g_Settings.AutoCalibration.stepSizeX = InSightFunctions.fnGetCellValue('Advanced.Calibration.StepSizeX')
  g_Settings.AutoCalibration.stepSizeY = InSightFunctions.fnGetCellValue('Advanced.Calibration.StepSizeY')

  g_Settings.AutoCalibration.calibRegionX = InSightFunctions.fnGetCellValue('Advanced.Calibration.AutoCalibRegion.X')
  g_Settings.AutoCalibration.calibRegionY = InSightFunctions.fnGetCellValue('Advanced.Calibration.AutoCalibRegion.Y')
  g_Settings.AutoCalibration.calibRegionHight = InSightFunctions.fnGetCellValue('Advanced.Calibration.AutoCalibRegion.Hight')
  g_Settings.AutoCalibration.calibRegionWidth = InSightFunctions.fnGetCellValue('Advanced.Calibration.AutoCalibRegion.Width')
  g_Settings.AutoCalibration.calibRegionAngle = InSightFunctions.fnGetCellValue('Advanced.Calibration.AutoCalibRegion.Angle')
  
  WriteCameraConfigToSheet()
};
function WriteCameraConfigToSheet() {
  if (g_Settings.LogImageFtp != null) {
    writeCellValue('LogImage.Enabled', g_Settings.LogImageFtp.Enabled)
    writeCellValue('LogImage.LogTrainImages', g_Settings.LogImageFtp.LogTrainImages)
    writeCellValue('LogImage.LogProductionImages', g_Settings.LogImageFtp.LogProductionImages)
    writeCellValue('LogImage.LogFailImages', g_Settings.LogImageFtp.LogFailImages)
    writeCellValue('LogImage.LogGraphics', g_Settings.LogImageFtp.LogGraphics)
      

    writeCellValue('LogImage.Password', g_Settings.LogImageFtp.Password)
    writeCellValue('LogImage.UserName', g_Settings.LogImageFtp.UserName)
    writeCellValue('LogImage.Hostname', g_Settings.LogImageFtp.Hostname)        
  }

  if (g_Settings.LogImageFtp.hasOwnProperty('UseTimeInFileName')) {
    writeCellValue('LogImage.UseTimeInFileName', g_Settings.LogImageFtp.UseTimeInFileName)    
    writeCellValue('LogImage.UseDateInFileName', g_Settings.LogImageFtp.UseDateInFileName)

    writeCellValue('LogImage.Resolution', Resolutions[g_Settings.LogImageFtp.ImageResolution])    
  }

  if (g_Settings.LogDataFtp != null) {   

    writeCellValue('Logging.FTP.Enabled', g_Settings.LogDataFtp.Enabled)
    writeCellValue('Logging.FTP.Password', g_Settings.LogDataFtp.Password)
    writeCellValue('Logging.FTP.UserName', g_Settings.LogDataFtp.UserName)
    writeCellValue('Logging.FTP.Hostname', g_Settings.LogDataFtp.Hostname)    
  }

  if (g_Settings.AutoCalibration != null) {
    writeCellValue('Advanced.Calibration.NumStepsX', g_Settings.AutoCalibration.NumStepsX)
    writeCellValue('Advanced.Calibration.NumStepsY', g_Settings.AutoCalibration.NumStepsY)
    writeCellValue('Advanced.Calibration.AutoStepSize', g_Settings.AutoCalibration.AutoStepSize)
    writeCellValue('Advanced.Calibration.MovingRangeMinX', g_Settings.AutoCalibration.MovingRangeMinX)
    writeCellValue('Advanced.Calibration.MovingRangeMaxX', g_Settings.AutoCalibration.MovingRangeMaxX)
    writeCellValue('Advanced.Calibration.MovingRangeMinY', g_Settings.AutoCalibration.MovingRangeMinY)
    writeCellValue('Advanced.Calibration.MovingRangeMaxY', g_Settings.AutoCalibration.MovingRangeMaxY)
    writeCellValue('Advanced.Calibration.NumStepsRotation', g_Settings.AutoCalibration.NumStepsRotation)
    writeCellValue('Advanced.Calibration.AngleMin', g_Settings.AutoCalibration.AngleMin)
    writeCellValue('Advanced.Calibration.AngleMax', g_Settings.AutoCalibration.AngleMax)
    writeCellValue('Advanced.Calibration.CantileverCompensation', g_Settings.AutoCalibration.CantileverCompensation)
    writeCellValue('Advanced.Calibration.StepSizeX', g_Settings.AutoCalibration.stepSizeX)
    writeCellValue('Advanced.Calibration.StepSizeY', g_Settings.AutoCalibration.stepSizeY)
    writeCellValue('Advanced.Calibration.FirstStepSize', g_Settings.AutoCalibration.FirstStepSize)
  }

  if (g_Settings.CustomCalibration != null) {
    writeCellValue('CustomCalibration.Enabled', g_Settings.CustomCalibration.Enabled)
  }
  let uncalibrated = readCellValue('HeCalibration.RunUncalibrated')
  if (uncalibrated != g_Settings.RunUncalibrated) {
    writeCellValue('HeCalibration.RunUncalibrated', g_Settings.RunUncalibrated)
  }  
  writeCellValue('Internal.Sensor.Name', g_Settings.SensorName)
 };


//* ***************************************************************************/
// Recipe Tables
//* ***************************************************************************/

const DefaultRecipeTables = {
  'Default': {
    'Recipe.PartTable.1': { "PartID": 1, "Description": "Part 1", "Moving": false, "FeatureIDs": [1] },
    'Recipe.PartTable.2': '',
    'Recipe.PartTable.3': '',
    'Recipe.PartTable.4': '',
    'Recipe.PartTable.5': '',
    'Recipe.PartTable.6': '',
    'Recipe.PartTable.7': '',
    'Recipe.PartTable.8': '',
    'Recipe.PartTable.9': '',
    'Recipe.PartTable.10': '',
    'Recipe.PartTable.11': '',
    'Recipe.PartTable.12': '',

    'Recipe.FeatureTable.1': { 'FeatureID': 1, 'Description': '', 'CameraID': 1, 'CamFeatureID': 1, 'PartID': 1 },
    'Recipe.FeatureTable.2': '',
    'Recipe.FeatureTable.3': '',
    'Recipe.FeatureTable.4': '',
    'Recipe.FeatureTable.5': '',
    'Recipe.FeatureTable.6': '',
    'Recipe.FeatureTable.7': '',
    'Recipe.FeatureTable.8': '',
    'Recipe.FeatureTable.9': '',
    'Recipe.FeatureTable.10': '',
    'Recipe.FeatureTable.11': '',
    'Recipe.FeatureTable.12': '',

    'Recipe.StepTable.1': { "StepID": 1, "Description": "P1 F1", "ShuttlingPoses": [1], "FeatureIDs": [1], "ExpSettings": [1] },
    'Recipe.StepTable.2': '',
    'Recipe.StepTable.3': '',
    'Recipe.StepTable.4': '',
    'Recipe.StepTable.5': '',
    'Recipe.StepTable.6': '',
    'Recipe.StepTable.7': '',
    'Recipe.StepTable.8': '',
    'Recipe.StepTable.9': '',
    'Recipe.StepTable.10': '',
    'Recipe.StepTable.11': '',
    'Recipe.StepTable.12': ''
  }
}
function RecipeTables(myIndex, usedCameras) {
  this.myIndex = myIndex
  this.usedCameras = usedCameras
  this.parts = {}
  this.steps = {}
  this.features = {}
  this.stepLookup = {}
  this.partLookup = {}
  this.cameraLookup = {}
};

RecipeTables.prototype.readFromSheet = function () {
  tracer.addMessage('Reading the Recipe Tables!')
  this.parts = {}
  this.steps = {}
  this.features = {}

  g_FeaturesInfos = {}
  g_RuntimeFeatures = {}
  g_TrainedFeatures = {}
  g_TrainedRobotPoses = {}
  g_GripCorrections = {}
  g_FrameCorrections = {}

  // Read the feature-table
  for (let i = 1; i <= RECIPE_MAX_FEATURES; i++) {
    let obj = {}
    let val = ''
    let path = RECIPE_FEATURETABLE + '.' + i.toString()
    if (checkTagNameAvailable(path) === true) {
      val = InSightFunctions.fnGetCellValue(path)
      obj = checkAndGetObjectFromJsonString(val)

      if (obj != null) {
        let index = obj['FeatureID']
        this.features[index] = obj

        g_FeaturesInfos[index] = new FeatureInfo(false, false, 1, 1)
        g_RuntimeFeatures[index] = new Feature(i, 0, 0, 0)

        if ((g_StoredTrainedFeatures != null) && (g_StoredTrainedFeatures.hasOwnProperty(index) == true)) {
          g_TrainedFeatures[index] = g_StoredTrainedFeatures[index]
        } else {
          g_TrainedFeatures[index] = []
          for (let g = 0; g < MAX_GRIPPERS; g++) {
            g_TrainedFeatures[index][g] = new Feature(0, 0, 0, 0)
          }
        }
      }
    }
  }
  g_Parts = {}

  for (let i = 1; i <= RECIPE_MAX_PARTS; i++) {
    let obj = {}
    let val = ''
    let path = RECIPE_PARTTABLE + '.' + i.toString()
    if (checkTagNameAvailable(path) === true) {
      val = InSightFunctions.fnGetCellValue(path)

      obj = checkAndGetObjectFromJsonString(val)

      if (obj != null) {
        let index = obj['PartID']
        this.parts[index] = obj

        g_Parts[index] = new Part()

        if ((g_StoredTrainedRobotPoses != null) && (g_StoredTrainedRobotPoses.hasOwnProperty(index) == true)) {
          g_TrainedRobotPoses[index] = g_StoredTrainedRobotPoses[index]
        } else {
          g_TrainedRobotPoses[index] = []
          for (let g = 0; g < MAX_GRIPPERS; g++) {
            g_TrainedRobotPoses[index][g] = new RobotPose(0, 0, 0, 0, 0, 0, 0)
          }
        }
        g_GripCorrections[index] = []
        for (let g = 0; g < MAX_GRIPPERS; g++) {
          g_GripCorrections[index][g] = new RobotPose(0, 0, 0, 0, 0, 0, 0)
        }

        g_FrameCorrections[index] = new RobotPose(0, 0, 0, 0, 0, 0, 0)

        g_Parts[index]['trainedRobotPose'] = g_TrainedRobotPoses[index]
        g_Parts[index]['gripCorrection'] = g_GripCorrections[index]
        g_Parts[index]['frameCorrection'] = g_FrameCorrections[index]

        for (var f = 0; f < obj['FeatureIDs'].length; f++) {
          g_Parts[index].runtimeFeatures.push(g_RuntimeFeatures[obj['FeatureIDs'][f]])
          g_Parts[index].trainedFeatures.push(g_TrainedFeatures[obj['FeatureIDs'][f]])
          g_Parts[index].featuresInfos.push(g_FeaturesInfos[obj['FeatureIDs'][f]])
          g_FeaturesInfos[obj['FeatureIDs'][f]]['partID'] = index
        }
      }
    }
  }

  for (var i = 1; i <= RECIPE_MAX_STEPS; i++) {
    var obj = {}
    var path = RECIPE_STEPTABLE + '.' + i.toString()
    if (checkTagNameAvailable(path) === true) {
      obj = checkAndGetObjectFromJsonString(InSightFunctions.fnGetCellValue(path))

      if (obj != null) {
        let index = obj['StepID']
        this.steps[index] = obj
      }
    }
  }

  this.createStepLookup()
  this.createPartLookup()
  this.createCameraLookup()
}

RecipeTables.prototype.createStepLookup = function () {
  tracer.addMessage('Create Step-Look-Up-Table')
  let lookUp = {}

  for (let i in this.steps) {
    var step = this.steps[i]

    let stepID = step['StepID']
    let featureIDs = step['FeatureIDs']
    let shuttlingPoses = step['ShuttlingPoses']
    let expSettingsIDs = step['ExpSettings']

    step = new StepLookupInfo()
    for (let i = 0; i < featureIDs.length; i++) {
      let feature = this.features[featureIDs[i]]
      let partID = feature['PartID']
      let part = this.parts[partID]
      let isMoving = part['Moving']
      let cameraID = feature['CameraID']
      let camFeatureID = feature['CamFeatureID']
      step.FeatureIDs.push(featureIDs[i])
      step.PartIDs.push(partID)

      step['Cam_' + cameraID.toString()]['IsMoving'] = isMoving
      step['Cam_' + cameraID.toString()]['Enabled'] = 1
      step['Cam_' + cameraID.toString()]['Feature_' + camFeatureID.toString()] = featureIDs[i]
      step['Cam_' + cameraID]['FeatureMask'] = step['Cam_' + cameraID]['FeatureMask'] | ((!!step['Cam_' + cameraID]['Feature_' + camFeatureID]) << (camFeatureID - 1))

      if (shuttlingPoses.length == featureIDs.length) {
        step['Cam_' + cameraID.toString()]['ShuttlePose'] = shuttlingPoses[i]
      } else {
        step['Cam_' + cameraID.toString()]['ShuttlePose'] = shuttlingPoses[0]
      }

      if (expSettingsIDs.length == featureIDs.length) {
        step['Cam_' + cameraID.toString()]['ExpSetting'] = expSettingsIDs[i]
      } else {
        step['Cam_' + cameraID.toString()]['ExpSetting'] = expSettingsIDs[0]
      }

      g_FeaturesInfos[featureIDs[i]].partIsMoving = isMoving
      g_FeaturesInfos[featureIDs[i]].shuttlePose = step['Cam_' + cameraID.toString()]['ShuttlePose']

      if ((cameraID != this.myIndex) && (this.usedCameras.indexOf(cameraID) >= 0)) {
        step.SendToSlave = 1
      }
    };
    lookUp[stepID] = step
  }

  this.stepLookup = lookUp
  g_Steps = lookUp
}

RecipeTables.prototype.createPartLookup = function () {
  tracer.addMessage('Create Part-Look-Up-Table')
  let partLookup = {}
  
  for (let p in this.parts) {
    
    var part = new PartLookupInfo()
    part.FeatureIDs = this.parts[p]['FeatureIDs']
    part.MPs = this.parts[p]['MPs']
    part.PartIsMoving = this.parts[p]['Moving']

    for (let f = 0; f < part.FeatureIDs.length; f++) {
      let feature = this.features[part.FeatureIDs[f]]
      let cameraID = feature['CameraID']
      if ((cameraID != this.myIndex) && (this.usedCameras.indexOf(cameraID) >= 0)) {
        part.SendToSlave = 1
      }

      let camFeatureID = feature['CamFeatureID']
      part['Cam_' + cameraID]['Feature_' + camFeatureID] = part.FeatureIDs[f]
      part['Cam_' + cameraID]['Enabled'] = 1
      part['Cam_' + cameraID]['FeatureMask'] = part['Cam_' + cameraID]['FeatureMask'] | ((!!part['Cam_' + cameraID]['Feature_' + camFeatureID]) << (camFeatureID - 1))

      for (let s in this.steps) {
        let featureIDs = this.steps[s]['FeatureIDs']
        let shuttlingPoses = this.steps[s]['ShuttlingPoses']
        let expSettings = this.steps[s]['ExpSettings']

        for (let i = 0; i < featureIDs.length; i++)
        {
          if (compareContentOfArrays(featureIDs, part['FeatureIDs'])) {
            if (shuttlingPoses.length == featureIDs.length) {
              part['Cam_' + cameraID.toString()]['ShuttlePose'] = shuttlingPoses[i]
            } else {
              part['Cam_' + cameraID.toString()]['ShuttlePose'] = shuttlingPoses[0]
            }

            if (expSettings.length == featureIDs.length) {
              part['Cam_' + cameraID.toString()]['ExpSetting'] = expSettings[i]
             
            } else {
              part['Cam_' + cameraID.toString()]['ExpSetting'] = expSettings[0]             
            }
          }
        }
      }
    }
    for (let c = 1; c <= MAX_CAMERAS; c++) {
      part['Cam_' + c]['IsMoving'] = part.PartIsMoving
    }
    partLookup[p] = part
  }
  this.partLookup = partLookup
}
RecipeTables.prototype.createCameraLookup = function () {
  let camLookup = {}

  for (var c = 0; c <= MAX_CAMERAS; c++) {
    camLookup[c] = new CameraLookupInfo()
    camLookup[c]['SendToSlave'] = 0
    if ((c != this.myIndex) && (this.usedCameras.indexOf(c) >= 0)) {
      camLookup[c]['SendToSlave'] = 1
    }
  }

  if ((this.usedCameras.length > 1) || (this.usedCameras[0] != this.myIndex)) {
    camLookup[0]['SendToSlave'] = 1
  }

  this.cameraLookup = camLookup
}
RecipeTables.prototype.setToDefault = function () {
  var recipe = DefaultRecipeTables.Default
  for (var k in recipe) {
    let val = ''
    if (typeof recipe[k] === 'object') {
      val = JSON.stringify(recipe[k])
    }
    writeCellValue(k, val)
  }
  this.readFromSheet()
}

function Camera() {
  this.Enabled = 0
  this.Feature_1 = 0
  this.Feature_2 = 0
  this.Feature_3 = 0
  this.Feature_4 = 0
  this.FeatureMask = 0
  this.ShuttlePose = 1
  this.ExpSetting = 1
  this.IsMoving = -560
};

function StepLookupInfo() {
  this.SendToSlave = 0
  this.FeatureIDs = []
  this.PartIDs = []
  this.Cam_1 = new Camera()
  this.Cam_2 = new Camera()
};
function PartLookupInfo() {
  this.SendToSlave = 0
  this.PartIsMoving = 0
  this.FeatureIDs = []
  this.MPs = []      // Multi Parts

  this.Cam_1 = new Camera()
  this.Cam_2 = new Camera()
};
function CameraLookupInfo() {
  this.SendToSlave = 0
};
//* ***************************************************************************/
// Calibration
//* ***************************************************************************/

function Calibration(index) {
  this.calibrationData = new CalibrationData()
  this.calibration = null
  this.results = null
  this.runstatus = 0
  this.index = index
};
Calibration.prototype.computeCalibration = function () {
  if (this.calibrationData.count >= 5) {
    let so = SharedObjects.getInstance()

    let calibrationMath = so.getSharedObject('CalibrathionMath')

    let calib = calibrationMath.computeCalibration(this.calibrationData, InSightFunctions.fnGetCellValue('Internal.HRes'), InSightFunctions.fnGetCellValue('Internal.VRes'))
    if (calib.Runstatus == 1) {
      this.calibration = calib.Calibration
      this.results = calib.Results
      this.runstatus = calib.Runstatus
    }
  }
  this.writeCalibrationInfoToSheet()
}
Calibration.prototype.writeCalibrationInfoToSheet = function () {
  writeCellValue('HECalibration.' + this.index + '.IsCameraMoving', this.calibrationData.isMoving)
  if (this.calibration != null) {
    writeCellValue('HECalibration.' + this.index + '.Valid', this.runstatus)

    if (this.runstatus == 1) {
      let trans = new cogMath.cc2XformLinear()
      if (this.calibration['isCameraMoving_'] == true) {
        trans.setXform(this.results.Transforms.Stage2DFromImage2D.xform)
      } else {
        trans.setXform(this.results.Transforms.Home2DFromImage2D.xform)
      }
      let xScale = trans.xScale()
      let yScale = trans.yScale()

      let diagnostics = this.results.Diagnostics
      writeCellValue('HECalibration.' + this.index + '.MaxImage2D', diagnostics['OverallResidualsImage2D']['Max'])
      writeCellValue('HECalibration.' + this.index + '.RMSImage2D', diagnostics['OverallResidualsImage2D']['Rms'])
      writeCellValue('HECalibration.' + this.index + '.MaxHome2D', diagnostics['OverallResidualsHome2D']['Max'])
      writeCellValue('HECalibration.' + this.index + '.RMSHome2D', diagnostics['OverallResidualsHome2D']['Rms'])
      writeCellValue('HECalibration.' + this.index + '.PixelSizeX', xScale)
      writeCellValue('HECalibration.' + this.index + '.PixelSizeY', yScale)
    }
  } else {
    writeCellValue('HECalibration.' + this.index + '.Valid', 0)
    writeCellValue('HECalibration.' + this.index + '.MaxImage2D', -1)
    writeCellValue('HECalibration.' + this.index + '.RMSImage2D', -1)
    writeCellValue('HECalibration.' + this.index + '.MaxHome2D', -1)
    writeCellValue('HECalibration.' + this.index + '.RMSHome2D', -1)
    writeCellValue('HECalibration.' + this.index + '.PixelSizeX', 0)
    writeCellValue('HECalibration.' + this.index + '.PixelSizeY', 0)
  }
}
Calibration.prototype.loadCalibrationDataFromFile = function () {
  tracer.addMessage('-> Load calibration data from file')

  let heCalibFileName = InSightFunctions.fnGetCellValue('HECalibration.FileName')
  if (heCalibFileName.length == 0) {
    heCalibFileName = InSightFunctions.fnGetCellValue('HECalibration.DefaultFileName')
  }

  if (this.index > 1) {
    heCalibFileName = heCalibFileName + '_SP' + this.index.toString()
  }
  let loadResult = this.calibrationData.loadFromFile(heCalibFileName + '.json')
  tracer.addMessage('<- Load calibration data from file')
  return loadResult
}
Calibration.prototype.saveCalibrationDataToFile = function () {
  tracer.addMessage('-> Save calibration data to file')

  let heCalibFileName = InSightFunctions.fnGetCellValue('HECalibration.FileName')
  if (heCalibFileName.length == 0) {
    heCalibFileName = InSightFunctions.fnGetCellValue('HECalibration.DefaultFileName')
  }
  if (this.index > 1) {
    heCalibFileName = heCalibFileName + '_SP' + this.index.toString()
  }
  this.calibrationData.saveToFile(heCalibFileName + '.json')
  tracer.addMessage('<- Save calibration data to file')
}
Calibration.prototype.deleteCalibrationDataFile = function () {
  tracer.addMessage('-> Delete calibration data file!')

  let heCalibFileName = InSightFunctions.fnGetCellValue('HECalibration.FileName')
  if (heCalibFileName.length == 0) {
    heCalibFileName = InSightFunctions.fnGetCellValue('HECalibration.DefaultFileName')
  }
  if (this.index > 1) {
    heCalibFileName = heCalibFileName + '_SP' + this.index.toString()
  }  
  this.calibrationData.saveToFile(heCalibFileName + '.json')
  cogUtils.deleteFile(heCalibFileName + '.json')
  tracer.addMessage('<- Delete calibration data file!')
}

function CalibrationData() {
  this.targetX = []
  this.targetY = []
  this.targetTheta = []
  this.targetValid = []
  this.robotX = []
  this.robotY = []
  this.robotTheta = []
  this.count = 0
  this.isMoving = 0
};
CalibrationData.prototype.init = function (data) {
  this.targetX = data['TargetX']
  this.targetY = data['TargetY']
  this.targetTheta = data['TargetTheta']
  this.targetValid = data['TargetValid']
  this.robotX = data['RobotX']
  this.robotY = data['RobotY']
  this.robotTheta = data['RobotTheta']
  this.count = data['Count']
  if (data.hasOwnProperty('isMoving')) {
    this.isMoving = data['isMoving']
  }
}
CalibrationData.prototype.initIdentity = function () {
    this.targetX = []
    this.targetY = []
    this.targetTheta = []
    this.targetValid = []
    this.robotX = []
    this.robotY = []
    this.robotTheta = []
    this.count = 0
    this.isMoving = 0

    for (let y = 0; y < 3; y++) {
        for (let x = 0; x < 3; x++) {
            this.targetX.push(x)
            this.targetY.push(y)
            this.targetTheta.push(0)
            this.targetValid.push(1)
            this.robotX.push(x)
            this.robotY.push(y)
            this.robotTheta.push(0)
        }
    }
    for (let theta = -10; theta < 20; theta += 10) {
        this.targetX.push(1)
        this.targetY.push(1)
        this.targetTheta.push(theta)
        this.targetValid.push(1)
        this.robotX.push(1)
        this.robotY.push(1)
        this.robotTheta.push(theta)
    }
    this.count = this.targetX.length
}

CalibrationData.prototype.loadFromFile = function (filename) {
  LoadCameraConfigFile()

  let fileObj = cogUtils.loadFile(filename)
  let result = 0

  this.reset()
  if (g_Settings.RunUncalibrated == 1) {
    this.initIdentity()
    result = 1
  }
  else if (fileObj != 'undefined') {
      this.init(fileObj)
      result = 1
  }

  return result
}

CalibrationData.prototype.saveToFile = function (filename) {
  let data = {}

  data['TargetX'] = this.targetX
  data['TargetY'] = this.targetY
  data['TargetTheta'] = this.targetTheta
  data['TargetValid'] = this.targetValid
  data['RobotX'] = this.robotX
  data['RobotY'] = this.robotY
  data['RobotTheta'] = this.robotTheta
  data['Count'] = this.count
  data['isMoving'] = Number(this.isMoving)
  cogUtils.saveFile(filename, data)
}
CalibrationData.prototype.reset = function () {
  this.targetX = []
  this.targetY = []
  this.targetTheta = []
  this.targetValid = []
  this.robotX = []
  this.robotY = []
  this.robotTheta = []
  this.count = 0
}

function Calibrations() {
  this.calibrations = { '1': new Calibration(1), '2': new Calibration(2) }
};
Calibrations.prototype.loadCalibrationsFromFile = function () {
  tracer.addMessage('loadCalibrationsFromFile')
  for (var i = 1; i <= MAX_CALIBRATIONS; i++) {
    if (this.calibrations[i] != null) {
      let exists = this.calibrations[i].loadCalibrationDataFromFile()
      if (exists <= 0) {
        this.calibrations[i] = null
      }
    }
  }
  this.computeCalibrations()
}
Calibrations.prototype.computeCalibrations = function () {
  for (var i in this.calibrations) {
    if (this.calibrations[i] != null) {
      this.calibrations[i].computeCalibration()
    }
  }
}
Calibrations.prototype.doAutoCalibration = function (robX, robY, robZ, robA, robB, robC, targetX, targetY, targetAngle, targetValid) {
  var acData = g_AutoCalibRuntime
  var nextRobotPose = { 'Valid': 0 }
  var inRange = 1

  if (acData.PreCalibPoses.length > 0) {
    inRange = inRange && Math.abs(acData.LastNextPos.X - robX).between(0, 0.1, true)
    inRange = inRange && Math.abs(acData.LastNextPos.Y - robY).between(0, 0.1, true)
    inRange = inRange && Math.abs(acData.LastNextPos.Z - robZ).between(0, 0.1, true)
    inRange = inRange && Math.abs(acData.LastNextPos.A - robA).checkAngleDelta(0.1, true)
    inRange = inRange && Math.abs(acData.LastNextPos.B - robB).checkAngleDelta(0.1, true)
    inRange = inRange && Math.abs(acData.LastNextPos.C - robC).checkAngleDelta(0.1, true)
  }

  if (inRange == 1) {
    if (acData.PreCalibration.Calibrated == 0) {
      if (acData.PreCalibPoses.length < 4) {
        switch (acData.PreCalibPoses.length) {
          case 0: nextRobotPose = this.firstPosition(acData, targetX, targetY, targetAngle, targetValid, robX, robY, robA); break
          case 1: nextRobotPose = this.secondPosition(acData, targetX, targetY, targetAngle, targetValid, robX, robY, robA); break
          case 2: nextRobotPose = this.thirdPosition(acData, targetX, targetY, targetAngle, targetValid, robX, robY, robA); break
          case 3: nextRobotPose = this.fourthPosition(acData, targetX, targetY, targetAngle, targetValid, robX, robY, robA); break
          default: console.log('Error in ACxxx Command'); break
        }
      }
      if (nextRobotPose != 0) {
        if ((acData.PreCalibPoses.length == 4) || ((acData.PreCalibPoses.length == 3) && (!acData.CantileverCompensation))) {
          console.log('Calc Pre-Calibration')
          var calibration = InSightFunctions.fnCalibrateAdvanced(acData.PreCalibPoses[0][0], acData.PreCalibPoses[0][1], acData.PreCalibPoses[0][3], acData.PreCalibPoses[0][4],
            acData.PreCalibPoses[1][0], acData.PreCalibPoses[1][1], acData.PreCalibPoses[1][3], acData.PreCalibPoses[1][4],
            acData.PreCalibPoses[2][0], acData.PreCalibPoses[2][1], acData.PreCalibPoses[2][3], acData.PreCalibPoses[2][4])

          if (acData.PreCalibPoses.length == 4) {
            var targetWorld_0 = InSightFunctions.fnTransPixelToWorld(calibration, acData.PreCalibPoses[0][0], acData.PreCalibPoses[0][1])
            var targetWorld_Rot = InSightFunctions.fnTransPixelToWorld(calibration, acData.PreCalibPoses[3][0], acData.PreCalibPoses[3][1])

            acData.angleCompX = (targetWorld_0.x - targetWorld_Rot.x) / acData.rotAngle
            acData.angleCompY = (targetWorld_0.y - targetWorld_Rot.y) / acData.rotAngle
          } else {
            acData.angleCompX = 0
            acData.angleCompY = 0
          }

          acData.PreCalibration.Calibrated = 1
          acData.PreCalibration.Calibration = calibration
        }
      }
    }
    if (acData.PreCalibration.Calibrated == 1) {
      var stepsX = acData.NumStepsX
      var stepsY = acData.NumStepsY
      var stepsRot = acData.NumStepsRotation
      var startX = acData.MovingRangeMinX
      var startY = acData.MovingRangeMinY
      var stepSizeX = (acData.MovingRangeMaxX - acData.MovingRangeMinX) / (acData.NumStepsX - 1)
      var stepSizeY = (acData.MovingRangeMaxY - acData.MovingRangeMinY) / (acData.NumStepsY - 1)
      var innerDist = acData.innerDist

      if (acData.CalibPoints.length == 0) {
        var innerDistXPix = innerDist
        var innerDistYPix = innerDist
        var centerX = 0
        var centerY = 0

        var robAngle = acData.PreCalibPoses[0][5]
        var compX = acData.angleCompX
        var compY = acData.angleCompY

        if (acData.AutoStepSize) {
          var xWidth = (acData.calibRegionHight - innerDistXPix) / (stepsX - 1)
          var yWidth = (acData.calibRegionWidth - innerDistYPix) / (stepsY - 1)
          var offsetX = (innerDistXPix) / 2
          var offsetY = (innerDistYPix) / 2
          var angle = acData.calibRegionAngle

          for (var x = 0; x < stepsX; x++) {
            for (var y = 0; y < stepsY; y++) {
              let point = InSightFunctions.fnPoint(acData.calibRegionX, acData.calibRegionY, angle, offsetX + (x * xWidth), offsetY + (y * yWidth), 2)

              centerX += point.x
              centerY += point.y

              var ppoint = InSightFunctions.fnPlotPoint(point.x, point.y, 'a', 5, 1)
              acData.CalibPoints.push(ppoint)

              let robPose = InSightFunctions.fnTransPixelToWorld(acData.PreCalibration.Calibration, point.x, point.y, 0)
              acData.RobCalibPoses.push([robPose.x, robPose.y, robAngle])
            }
          }
          centerX = centerX / (stepsX * stepsY)
          centerY = centerY / (stepsX * stepsY)

          let point = InSightFunctions.fnPoint(0, 0, 0, centerX, centerY, 2)
          acData.CalibPoints.push(point)

          // stepsRot, angleMin, angleMax
          var angleRange = acData.AngleMax - acData.AngleMin
          var angleStepSize = angleRange / (stepsRot - 1)

          for (var r = 0; r < stepsRot; r++) {
            var rot = acData.AngleMin + (r * angleStepSize)

            var cX = rot * compX
            var cY = rot * compY

            myLog('Compensation: ' + cX + ' / ' + cY)

            var robotR = robAngle + rot
            if (robotR > 180.0) {
              robotR = (robotR - 360)
            } else
              if (robotR < -180.0) {
                robotR = (robotR + 360)
              }

            let robPose = InSightFunctions.fnTransPixelToWorld(acData.PreCalibration.Calibration, centerX, centerY, 0)
            acData.RobCalibPoses.push([robPose.x + cX, robPose.y + cY, robotR])
          }
        } else {
          var xWidth = stepSizeX
          var yWidth = stepSizeY
          var offsetX = 0
          var offsetY = 0
          var angle = 0

          var robStartX = acData.PreCalibPoses[0][3] + startX
          var robStartY = acData.PreCalibPoses[0][4] + startY

          for (var x = 0; x < stepsX; x++) {
            for (var y = 0; y < stepsY; y++) {
              acData.RobCalibPoses.push([robStartX + x * stepSizeX, robStartY + y * stepSizeY, robAngle])
            }
          }
          for (var i = 0; i < (stepsX * stepsY); i++) {
            centerX += acData.RobCalibPoses[i][0]
            centerY += acData.RobCalibPoses[i][1]
          }

          centerX = centerX / (stepsX * stepsY)
          centerY = centerY / (stepsX * stepsY)

          // stepsRot, angleMin, angleMax
          var angleRange = acData.AngleMax - acData.AngleMin
          var angleStepSize = angleRange / (stepsRot - 1)

          for (var r = 0; r < stepsRot; r++) {
            let rot = acData.AngleMin + (r * angleStepSize)
            let cX = rot * compX
            let cY = rot * compY

            let robotR = robAngle + rot
            if (robotR > 180.0) {
              robotR = (robotR - 360)
            } else
              if (robotR < -180.0) {
                robotR = (robotR + 360)
              }
            acData.RobCalibPoses.push([centerX + cX, centerY + cY, robotR])
          }
        }
      }

      if (acData.stepCount < (stepsX * stepsY + stepsRot)) {
        nextRobotPose['NextX'] = acData.RobCalibPoses[acData.stepCount][0]
        nextRobotPose['NextY'] = acData.RobCalibPoses[acData.stepCount][1]
        nextRobotPose['NextAngle'] = acData.RobCalibPoses[acData.stepCount][2]
        nextRobotPose['Step'] = acData.stepCount
        nextRobotPose['Valid'] = 2
        acData.stepCount++
      } else
        if (acData.stepCount == (stepsX * stepsY + stepsRot)) {
          nextRobotPose['NextX'] = acData.PreCalibPoses[0][3]
          nextRobotPose['NextY'] = acData.PreCalibPoses[0][4]
          nextRobotPose['NextAngle'] = acData.PreCalibPoses[0][5]
          nextRobotPose['Step'] = acData.stepCount
          nextRobotPose['Valid'] = 2
          acData.stepCount++
        } else {
          nextRobotPose['NextX'] = acData.PreCalibPoses[0][3]
          nextRobotPose['NextY'] = acData.PreCalibPoses[0][4]
          nextRobotPose['NextAngle'] = acData.PreCalibPoses[0][5]
          nextRobotPose['Valid'] = 1
        }
    }
  } else {
    nextRobotPose['Valid'] = ECodes.E_NOT_GIVEN_CALIBRATION_POSE
  }

  if (nextRobotPose['Valid'] > 0) {
    acData.LastNextPos.X = nextRobotPose['NextX']
    acData.LastNextPos.Y = nextRobotPose['NextY']
    acData.LastNextPos.Z = robZ
    acData.LastNextPos.A = nextRobotPose['NextAngle']
    acData.LastNextPos.B = robB
    acData.LastNextPos.C = robC
  }
  return nextRobotPose
}
Calibrations.prototype.CheckData = function (data, isMoving) {
  if (data.count < MIN_POSES) {
    console.log('To few positions! ' + MIN_POSES + ' positions needed!')
    return ECodes.E_INVALID_CALIBRATION_DATA
  }

  if (data.hasOwnProperty('LastNextPos')) {
    delete data['LastNextPos']
  }

  var cntValid = 0
  var cntRotated = 0

  var minRobotAngle = data.robotTheta[0]
  var maxRobotAngle = data.robotTheta[0]

  for (var i = 0; i < data.count; i++) {
    if (data.targetValid[i] == 1) {
      cntValid++

      if (data.robotTheta[i] < minRobotAngle) {
        minRobotAngle = data.robotTheta[i]
        cntRotated++
      } else
        if (data.robotTheta[i] > maxRobotAngle) {
          maxRobotAngle = data.robotTheta[i]
          cntRotated++
        }
    }
  }

  var onStraight = true
  for (var t = 2; t < data.count; t++) {
    onStraight = this.CheckPointOnStraightLine(data.robotX[0], data.robotY[0], data.robotX[1], data.robotY[1], data.robotX[t], data.robotY[t])
    if (onStraight == false) { break }
  }

  if (onStraight == true) {
    console.log('All positions on a straight!')
    return 0
  }

  if ((cntValid) < MIN_POSES) {
    console.log('To few valid positions! ' + (MIN_POSES) + ' valid positions needed!')
    return 0
  }

  if ((cntRotated + 1) < MIN_POSES_ROTATED) {
    console.log('To few valid rotated positions! ' + (MIN_POSES_ROTATED) + 'rotated positions needed!')
    return 0
  }

  var angleRange = Math.abs(maxRobotAngle - minRobotAngle)
  if (angleRange < MIN_ANGLE_RANGE) {

    console.log('The angle range is to small! ' + (MIN_ANGLE_RANGE) + '° needed!')
    return 0
  }

  if (isMoving == 1) {
    data.isMoving = 1
  }

  console.log('Angle range: ' + minRobotAngle + ' / ' + maxRobotAngle + ' / ' + angleRange)
  console.log('Valid: ' + cntValid + ' / Rotated: ' + (cntRotated))

  return 1
}
Calibrations.prototype.CheckPointOnStraightLine = function (pX0, pY0, pX1, pY1, pX, pY) {
  var ret = false

  var dirX = pX1 - pX0
  var dirY = pY1 - pY0

  var rX = pX - pX0
  var rY = pY - pY0

  if ((dirX == 0) && (dirY == 0)) {
    ret = true
  } else {
    if ((dirX == 0) && (rX == 0)) {
      ret = true
    } else
      if ((dirY == 0) && (rY == 0)) {
        ret = true
      } else
        if (((pX - pX0) / dirX) == ((pY - pY0) / dirY)) {
          ret = true
        } else {
          ret = false
        }
  }

  return ret
}
Calibrations.prototype.checkMinDistance = function (x1, y1, x2, y2) {
  var ret = false
  var dist = this.computeDistance(x1, y1, x2, y2)
  if (dist >= 50) /* --------------------------this.minDist_Pixel */ {
    ret = true
  }
  return ret
}
Calibrations.prototype.computeDistance = function (x1, y1, x2, y2) {
  var dist = Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2))
  return dist
}
Calibrations.prototype.firstPosition = function (acData, targetX, targetY, targetAngle, targetValid, robX, robY, robAngle) {
  console.log('Add first Position')
  var nextRobotPose = { 'Valid': 0 }
  if (targetValid > 0) {
    // Add this point to the calibration points
    acData.PreCalibPoses.push([targetX, targetY, targetAngle, robX, robY, robAngle])

    nextRobotPose['NextX'] = robX + acData.lastMoveDistance_X
    nextRobotPose['NextY'] = robY
    nextRobotPose['NextAngle'] = robAngle
    nextRobotPose['Valid'] = 2
  }

  return nextRobotPose
}
Calibrations.prototype.secondPosition = function (acData, targetX, targetY, targetAngle, targetValid, robX, robY, robAngle) {
  console.log('Check second Position')
  var nextRobotPose = { 'Valid': 0 }
  var valid = 1

  if (targetValid > 0) {
    // Check min distance
    if (this.checkMinDistance(acData.PreCalibPoses[0][0], acData.PreCalibPoses[0][1], targetX, targetY) || !acData.AutoStepSize || (acData.loopCnt != 0)) {
      console.log('Add second Position')
      // Add this point to the calibration points
      acData.PreCalibPoses.push([targetX, targetY, targetAngle, robX, robY, robAngle])
      if (acData.loopCnt != 0) {
        acData.lastMoveDistance_Y = g_AutoCalibRuntime['FirstStepSize']// START_MOVE_DISTANCE_MM
      } else {
        acData.lastMoveDistance_Y = acData.lastMoveDistance_X
      }

      acData.lastMoveDistance_X = 0
    } else {      
      myLog('Compute an other second Position')

      var dist = this.computeDistance(acData.PreCalibPoses[0][0], acData.PreCalibPoses[0][1], targetX, targetY)
      var mmPerPix = acData.lastMoveDistance_X / dist
      
      acData.lastMoveDistance_X = mmPerPix * MIN_DIST_PIXEL * 1.1/* ---------------------------------minDist_Pixel*1.1; */
      acData.lastMoveDistance_Y = 0
    }
    acData.loopCnt = 0
    acData.Direction = 1
  } else {
    acData.loopCnt++

    if ((acData.loopCnt > MAX_LOOPS) && (acData.Direction == -1)) {
      valid = 0
    } else {
      console.log('No target found, reduce step size!')

      if ((acData.loopCnt > MAX_LOOPS) && (acData.Direction == 1)) {
        acData.loopCnt = 0
        acData.Direction = -1
        console.log('Reverse direction!')
      }

      console.log('Maybe out of FOV, reduce step size!')

      if ((acData.loopCnt == 0) && (acData.Direction == -1)) {
        acData.lastMoveDistance_X = g_AutoCalibRuntime['FirstStepSize'] * -1 //  START_MOVE_DISTANCE_MM * -1
      } else {
        acData.lastMoveDistance_X = acData.lastMoveDistance_X / 2
      }
      acData.lastMoveDistance_Y = 0
    }
  }

  if (valid > 0) {
    nextRobotPose['NextX'] = acData.PreCalibPoses[0][3] + acData.lastMoveDistance_X
    nextRobotPose['NextY'] = acData.PreCalibPoses[0][4] + acData.lastMoveDistance_Y
    nextRobotPose['NextAngle'] = acData.PreCalibPoses[0][5]
    nextRobotPose['Valid'] = 2
  } else {
    console.log('Max loops done! No target found!')
    nextRobotPose['Valid'] = ECodes.E_CALIBRATION_FAILED
  }
  return nextRobotPose
}
Calibrations.prototype.thirdPosition = function (acData, targetX, targetY, targetAngle, targetValid, robX, robY, robAngle) {
  console.log('Check third Position')
  var nextRobotPose = { 'Valid': 0 }
  var rot = 0
  var valid = 1
  if (targetValid > 0) {
    // Check min distance
    if (this.checkMinDistance(acData.PreCalibPoses[0][0], acData.PreCalibPoses[0][1], targetX, targetY) || !acData.AutoStepSize || (acData.loopCnt != 0)) {
      console.log('Add third Position')
      // Add this point to the calibration points
      acData.PreCalibPoses.push([targetX, targetY, targetAngle, robX, robY, robAngle])

      // Nothing more to do
      acData.lastMoveDistance_Y = 0
      acData.lastMoveDistance_X = 0
      rot = acData.rotAngle
    } else {
      console.log('Compute an other Distance Position')
      var dist = Math.sqrt(Math.pow(acData.PreCalibPoses[0][0] - targetX, 2) + Math.pow(acData.PreCalibPoses[0][1] - targetY, 2))
      var mmPerPix = acData.lastMoveDistance_Y / dist
      acData.lastMoveDistance_Y = mmPerPix * MIN_DIST_PIXEL * 1.1 /* ------------------this.minDist_Pixel */
      acData.lastMoveDistance_X = 0
    }
    acData.loopCnt = 0
    acData.Direction = 1
  } else {
    acData.loopCnt++
    if ((acData.loopCnt > MAX_LOOPS) && (acData.Direction == -1)) {
      valid = 0
    } else {
      console.log('No target found, reduce step size!')

      if ((acData.loopCnt > MAX_LOOPS) && (acData.Direction == 1)) {
        acData.loopCnt = 0
        acData.Direction = -1
        console.log('Reverse direction!')
      }

      console.log('Maybe out of FOV, reduce step size!')

      if ((acData.loopCnt == 0) && (acData.Direction == -1)) {
        acData.lastMoveDistance_Y = g_AutoCalibRuntime['FirstStepSize'] * -1 //  START_MOVE_DISTANCE_MM * -1
      } else {
        acData.lastMoveDistance_Y = acData.lastMoveDistance_Y / 2
      }
    }
  }

  if (valid > 0) {
    nextRobotPose['NextX'] = acData.PreCalibPoses[0][3] + acData.lastMoveDistance_X
    nextRobotPose['NextY'] = acData.PreCalibPoses[0][4] + acData.lastMoveDistance_Y
    var r = acData.PreCalibPoses[0][5] + rot
    if (r > 180.0) {
      r = (r - 360)
    } else
      if (r < -180.0) {
        r = (r + 360)
      }

    nextRobotPose['NextAngle'] = r
    nextRobotPose['Valid'] = 2
  } else {
    console.log('Max loops done! No target found!')
    nextRobotPose['Valid'] = ECodes.E_CALIBRATION_FAILED
  }

  return nextRobotPose
}
Calibrations.prototype.fourthPosition = function (acData, targetX, targetY, targetAngle, targetValid, robX, robY, robAngle) {
  console.log('Check fourth Position')
  var nextRobotPose = { 'Valid': 0 }
  if (targetValid > 0) {
    // Add this point to the calibration points
    acData.PreCalibPoses.push([targetX, targetY, targetAngle, robX, robY, robAngle])
    // compute the next robot pose

    nextRobotPose['NextX'] = acData.PreCalibPoses[0][3]
    nextRobotPose['NextY'] = acData.PreCalibPoses[0][4]
    nextRobotPose['NextAngle'] = acData.PreCalibPoses[0][5]
    nextRobotPose['Valid'] = 2
  }

  return nextRobotPose
}
Calibrations.prototype.doCustomCalibration = function (data, handedness, offsetX, offsetY) {
  var retData = data

  var hand = (handedness == 0 ? 1 : -1)
  if ((data.count == 2) || (data.count == 9)) {
    if (data.count == 2) {
      retData = this.create2(data, hand)
    } else {
      retData = data
    }

    retData = this.createRotated(retData, hand, offsetX, offsetY)
  }
  return retData
}
Calibrations.prototype.create2 = function (data, handedness) {
  
  // Motion vector
  var dx = data.robotX[1] - data.robotX[0]
  var dy = data.robotY[1] - data.robotY[0]
  // real motion vector
  var vm1 = new cogMath.Vector2(dx, dy)
  // orthogonal motion vector
  var vm2 = vm1.rotate(0.5 * Math.PI)

  // Vision vector
  var dCol = data.targetX[1] - data.targetX[0]
  var dRow = data.targetY[1] - data.targetY[0]
  // real vision moving vector
  var vv1 = new cogMath.Vector2(dCol, dRow)
  // orthogonal vision moving vector
  var vv2 = vv1.rotate(handedness * 0.5 * Math.PI)

  // First HECalib positions
  var posM = new cogMath.Vector2(data.robotX[0], data.robotY[0]) // 1st motion position
  var posV = new cogMath.Vector2(data.targetX[0], data.targetY[0]) // 1st vision position

  // Now move motion and vision positions in a 3 x 3 matrix
  for (var dir2 = 0; dir2 < 3; dir2++) { // direction2 loop
    var v = vm2.scale(dir2)
    var vmStart = posM.add(v)

    v = vv2.scale(dir2)
    var vvStart = posV.add(v)

    for (var dir1 = 0; dir1 < 3; dir1++) { // direction1 loop
      v = vm1.scale(dir1)
      var vm = vmStart.add(v)

      v = vv1.scale(dir1)
      var vv = vvStart.add(v)

      data.targetX.push(vv.x)
      data.targetY.push(vv.y)
      data.targetValid.push(1)
      data.targetTheta.push(0)

      data.robotX.push(vm.x)
      data.robotY.push(vm.y)
      data.robotTheta.push(0)

      data.count++
    }
  }

  data.targetX.splice(0, 2)
  data.targetY.splice(0, 2)
  data.targetValid.splice(0, 2)
  data.targetTheta.splice(0, 2)

  data.robotX.splice(0, 2)
  data.robotY.splice(0, 2)
  data.robotTheta.splice(0, 2)

  data.count = data.targetX.length

  return data
}

Calibrations.prototype.createRotated = function (data, handedness, offsetX, offsetY) {
  // Create a 3-Point calibration from matrix position (0,0), (2,0) and (2,2)
  // var retData = data;
  var calibPoints = []
  var cpt = {}
  cpt.Vx = data.targetX[0]
  cpt.Vy = data.targetY[0]
  cpt.Mx = data.robotX[0] + offsetX
  cpt.My = data.robotY[0] + offsetY
  calibPoints.push(cpt)
  cpt = {}
  cpt.Vx = data.targetX[2]
  cpt.Vy = data.targetY[2]
  cpt.Mx = data.robotX[2] + offsetX
  cpt.My = data.robotY[2] + offsetY
  calibPoints.push(cpt)
  cpt = {}
  cpt.Vx = data.targetX[8]
  cpt.Vy = data.targetY[8]
  cpt.Mx = data.robotX[8] + offsetX
  cpt.My = data.robotY[8] + offsetY
  calibPoints.push(cpt)

  var calib = InSightFunctions.fnCalibrateAdvanced(calibPoints[0].Vx, calibPoints[0].Vy, calibPoints[0].Mx, calibPoints[0].My,
    calibPoints[1].Vx, calibPoints[1].Vy, calibPoints[1].Mx, calibPoints[1].My,
    calibPoints[2].Vx, calibPoints[2].Vy, calibPoints[2].Mx, calibPoints[2].My)

  // Rotated positions
  // Vector Origin -> Feature in motion space:
  var vFeature0 = new cogMath.Vector2(offsetX, offsetY)
  var vRotPoint = new cogMath.Vector2(data.robotX[0], data.robotY[0])

  for (var rot = -15; rot < 20; rot += 30) {
    var vFeatureRot = vFeature0.rotate(handedness * Math.PI * rot / 180.0) // Rotated feature pos in world coords
    var vFeatureRotMoved = vFeatureRot.add(vRotPoint)
    var ptRot = InSightFunctions.fnTransWorldToPixel(calib, vFeatureRotMoved.x, vFeatureRotMoved.y, 0) // Rotated feature pos in pixels

    data.robotX.push(data.robotX[0])
    data.robotY.push(data.robotY[0])
    data.robotTheta.push(data.robotTheta[0] + rot)

    data.targetX.push(ptRot.row)
    data.targetY.push(ptRot.col)
    data.targetValid.push(1)
    data.targetTheta.push(rot)
    data.count++
  }
  return data
}
//* ***************************************************************************/
// Calculations
//* ***************************************************************************/
function ComputeAlignMode_1(partID, gripperID, resMode, robPose) {
  // TODO: nur gültige Kalibrierung zulassen

  let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)
  let rPose = new RobotPose(robPose.x, robPose.y, robPose.z, robPose.thetaZ, robPose.thetaY, robPose.thetaX, robPose.valid)
  //let rPose = new RobotPose(robPose.x, robPose.y, robPose.z, robPose.thetaX, robPose.thetaY, robPose.thetaZ, robPose.valid)

  if (!g_Parts.hasOwnProperty(partID)) {
    newRobPose.valid = ECodes.E_INVALID_PART_ID
    return newRobPose
  }
  let trainPoints = []
  let runPoints = []
  let state = ECodes.E_NO_ERROR
  let heCalib = null
  heCalib = g_Calibrations[1]['calibration']

  if (g_Parts[partID]['runtimeFeatures'].length == g_Parts[partID]['trainedFeatures'].length) {
    if(checkAllFeaturesOfPartTrained(partID, gripperID)) {
      if(checkAllRuntimeFeaturesOfPartValid(partID)) {   
        for (var i = 0; i < g_Parts[partID]['runtimeFeatures'].length; i++) {
          let tf = cloneObj(g_Parts[partID]['trainedFeatures'][i][gripperID])
          let rf = cloneObj(g_Parts[partID]['runtimeFeatures'][i])
          if (tf.valid > 0) {
            if (g_Parts[partID]['featuresInfos'][i]['partIsMoving'] == 1) {
              let homeFromHand_RT = new cogMath.cc2Rigid()
              homeFromHand_RT.setXform(robPose.thetaZ, [robPose.x, robPose.y])
              homeFromHand_RT = cogMath.convertToUncorrected(g_Calibrations[1]['results']['Transforms'], homeFromHand_RT)

              let rob = cloneObj(g_Parts[partID]['trainedRobotPose'][gripperID])
              let homeFromHand_TT = new cogMath.cc2Rigid()
              homeFromHand_TT.setXform(rob.thetaZ, [rob.x, rob.y])
              homeFromHand_TT = cogMath.convertToUncorrected(g_Calibrations[1]['results']['Transforms'], homeFromHand_TT)

              rPose.x = rob.x
              rPose.y = rob.y
              rPose.z = rob.z
              rPose.thetaX = rob.thetaX
              rPose.thetaY = rob.thetaY
              rPose.thetaZ = rob.thetaZ
              rPose.valid = rob.valid

              let taXf = new cogMath.cc2Rigid()
              taXf.setXform(tf.thetaInDegrees, [tf.x, tf.y])

              let raXf = new cogMath.cc2Rigid()
              raXf.setXform(rf.thetaInDegrees, [rf.x, rf.y])

              let rXf = new cogMath.cc2Rigid()
              rXf.setXform(rf.thetaInDegrees, [rf.x, rf.y])

              taXf = homeFromHand_TT.compose(taXf)
              raXf = homeFromHand_TT.compose(raXf)
  
              tf.x = taXf.trans()[0]
              tf.y = taXf.trans()[1]
              tf.thetaInDegrees = taXf.angleInDegrees()

              rf.x = raXf.trans()[0]
              rf.y = raXf.trans()[1]
              rf.thetaInDegrees = raXf.angleInDegrees()
            }
            trainPoints.push(new Feature(tf.x, tf.y, tf.thetaInDegrees, tf.valid))
            runPoints.push(new Feature(rf.x, rf.y, rf.thetaInDegrees, rf.valid))
          } else {
            state = ECodes.E_TARGET_POSE_NOT_TRAINED
          }
        }
      } else {
        state = ECodes.E_PART_NOT_ALL_FEATURES_LOCATED  
      }
    } else {
      state = ECodes.E_PART_NOT_ALL_FEATURES_LOCATED
    }

    if (state == ECodes.E_NO_ERROR) {
      let result = {}      
      //result = ComputeDesiredStagePosition(trainPoints, runPoints, heCalib, robPose.x, robPose.y, robPose.thetaZ)
      result = ComputeDesiredStagePosition(trainPoints, runPoints, heCalib, rPose.x, rPose.y, rPose.thetaZ)

      let abs = {}
      abs.X = result.desiredHome2DFromStage2DCorrected.trans()[0]
      abs.Y = result.desiredHome2DFromStage2DCorrected.trans()[1]
      abs.A = result.desiredHome2DFromStage2DCorrected.angleInDegrees()
      abs.Valid = 1

      let robTargetPos = new cogMath.cc2Rigid()
      robTargetPos.setXform(abs.A, [abs.X, abs.Y])

      let robPos = new cogMath.cc2Rigid()
      robPos.setXform(robPose.thetaZ, [robPose.x, robPose.y])

      if ((resMode == ResultMode.ABS) || (resMode == ResultMode[ResultMode.ABS])) {
        newRobPose.x = abs.X
        newRobPose.y = abs.Y
        newRobPose.z = robPose.z
        newRobPose.thetaZ = abs.A
        newRobPose.thetaY = robPose.thetaY
        newRobPose.thetaX = robPose.thetaX
        newRobPose.valid = 1
      } else
        if ((resMode == ResultMode.OFF) || (resMode == ResultMode[ResultMode.OFF])) {
        let offRes = new cogMath.cc2Rigid()
        offRes.setXform(robTargetPos.angleInDegrees() - robPos.angleInDegrees(), [robTargetPos.trans()[0] - robPos.trans()[0], robTargetPos.trans()[1] - robPos.trans()[1]])

        newRobPose.x = offRes.trans()[0]
        newRobPose.y = offRes.trans()[1]
        newRobPose.z = 0.0
        newRobPose.thetaZ = offRes.angleInDegrees()
        newRobPose.thetaY = 0.0
        newRobPose.thetaX = 0.0
        newRobPose.valid = 1
      } else {
        newRobPose.valid = ECodes.E_INVALID_ARGUMENT
      }
    } else {
      newRobPose.valid = state
    }
  }
  return newRobPose
};

function ComputeAlignMode_2(partID1, partID2, gripperID, resMode, robPose) {
  let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)

  if (!g_Parts.hasOwnProperty(partID1)) {
    newRobPose.valid = ECodes.E_INVALID_PART_ID
    return newRobPose
  }

  let trainedRobotPose = cloneObj(g_Parts[partID1]['trainedRobotPose'][gripperID]) // rtr
  if (trainedRobotPose.valid <= 0) {
    newRobPose.valid = ECodes.E_ROBOT_POSE_NOT_TRAINED
    return newRobPose
  }

  let runtimeFeatuers = g_Parts[partID1]['runtimeFeatures']
  let trainedFeatuers = g_Parts[partID1]['trainedFeatures']

  for (let tf = 0; tf < trainedFeatuers.length; tf++) {
    if (trainedFeatuers[tf][gripperID].valid <= 0) {
      newRobPose.valid = ECodes.E_TARGET_POSE_NOT_TRAINED
      return newRobPose
    }
  }
  let shuttlePose = g_Parts[partID1]['featuresInfos'][0]['shuttlePose']

  if ((runtimeFeatuers.length != trainedFeatuers.length) ||
    (runtimeFeatuers.length > 2) ||
    (runtimeFeatuers.length < 1)) {
    newRobPose.valid = ECodes.E_UNSPECIFIED
    return newRobPose
  }
  let length = runtimeFeatuers.length
  let sumRuntime = { 'x': 0, 'y': 0, 'theta': 0 } // trt -> target runtime
  let sumTrained = { 'x': 0, 'y': 0, 'theta': 0 } // ttr -> target trained

  for (let i = 0; i < length; i++) {
    sumRuntime.x += runtimeFeatuers[i].x
    sumRuntime.y += runtimeFeatuers[i].y
    sumRuntime.theta += runtimeFeatuers[i].thetaInDegrees

    sumTrained.x += trainedFeatuers[i][gripperID].x
    sumTrained.y += trainedFeatuers[i][gripperID].y
    sumTrained.theta += trainedFeatuers[i][gripperID].thetaInDegrees
  }

  if (length > 1) {
    sumRuntime.x /= length
    sumRuntime.y /= length
    sumRuntime.theta /= length

    sumTrained.x /= length
    sumTrained.y /= length
    sumTrained.theta /= length
    sumRuntime.theta = InSightFunctions.fnPointToPoint(runtimeFeatuers[0].x, runtimeFeatuers[0].y, runtimeFeatuers[1].x, runtimeFeatuers[1].y, 0).getAngle()

    sumTrained.theta = InSightFunctions.fnPointToPoint(trainedFeatuers[0][gripperID].x, trainedFeatuers[0][gripperID].y, trainedFeatuers[1][gripperID].x, trainedFeatuers[1][gripperID].y, 0).getAngle()
  }

  let calibXforms = g_Calibrations[shuttlePose].results.Transforms

  let trainResult = {}
  let runtimeResult = {}
  // Train
  {
    let homeFromTarget = new cogMath.cc2Rigid()
    homeFromTarget.setXform(sumTrained.theta, [sumTrained.x, sumTrained.y])
    trainResult['Home2DFromTarget2D'] = homeFromTarget

    var homeFromHand = new cogMath.cc2Rigid()
    homeFromHand.setXform(trainedRobotPose.thetaZ, [trainedRobotPose.x, trainedRobotPose.y])
    homeFromHand = cogMath.convertToCorrected(calibXforms, homeFromHand)
    trainResult['Home2DFromRobot2D'] = homeFromHand

    var tmpLinear = homeFromHand.inverse().compose(homeFromTarget)
    var handFromTarget = cogMath.lin2Rigid(tmpLinear)
    trainResult['Robot2DFromTarget2D'] = handFromTarget
  }
  // Runtime
  
  {
    var homeFromTarget = new cogMath.cc2Rigid()
    homeFromTarget.setXform(sumRuntime.theta, [sumRuntime.x, sumRuntime.y])
    runtimeResult['Home2DFromTarget2D'] = homeFromTarget

    // Step is trained, we are running the "XT" command
    var tmpLinear = new cogMath.cc2XformLinear()
    tmpLinear.setXform(trainResult['Robot2DFromTarget2D'].xform)
    var handFromTarget = cogMath.lin2Rigid(tmpLinear)

    var homeFromHandRT = new cogMath.cc2Rigid() 
    homeFromHandRT = homeFromTarget.compose(handFromTarget.inverse())

    var homeFromHandRT_u = cogMath.convertToUncorrected(calibXforms, homeFromHandRT)

    var robotAbsXYA = {}
    robotAbsXYA['A'] = homeFromHandRT_u.angleInDegrees()
    robotAbsXYA['X'] = homeFromHandRT_u.trans()[0]
    robotAbsXYA['Y'] = homeFromHandRT_u.trans()[1]
   
    // The offset position
    var homeFromHandTrained = new cogMath.cc2Rigid()
    tmpLinear.setXform(trainResult['Home2DFromRobot2D'].xform)
    homeFromHandTrained = cogMath.lin2Rigid(tmpLinear)

    // Convert to uncorrected motion space
    var homeFromHandTrained_u = cogMath.convertToUncorrected(calibXforms, homeFromHandTrained)

    var trainFromRT = new cogMath.cc2Rigid()
    trainFromRT = homeFromHandRT_u.compose(homeFromHandTrained_u.inverse())

    var robotOffXYA = {}
    robotOffXYA['A'] = homeFromHandRT_u.angleInDegrees() - homeFromHandTrained_u.angleInDegrees()
    robotOffXYA['X'] = homeFromHandRT_u.trans()[0] - homeFromHandTrained_u.trans()[0]
    robotOffXYA['Y'] = homeFromHandRT_u.trans()[1] - homeFromHandTrained_u.trans()[1]

    var robotFrameXYA = {}
    robotFrameXYA['A'] = trainFromRT.angleInDegrees()
    robotFrameXYA['X'] = trainFromRT.trans()[0]
    robotFrameXYA['Y'] = trainFromRT.trans()[1]

    // Picked
    var robotPickedXYA = {}
    var homeFromRobot = new cogMath.cc2Rigid()
    homeFromRobot.setXform(robPose.thetaZ, [robPose.x, robPose.y])

    var picked = new cogMath.cc2Rigid()
    picked = homeFromHand.inverse().compose(homeFromHandRT_u)

    var homeFromHandPicked = new cogMath.cc2Rigid()
    homeFromHandPicked = homeFromRobot.compose(picked.inverse())

    robotPickedXYA['A'] = homeFromHandPicked.angleInDegrees()
    robotPickedXYA['X'] = homeFromHandPicked.trans()[0]
    robotPickedXYA['Y'] = homeFromHandPicked.trans()[1]
  }

  if ((resMode == ResultMode.ABS) || (resMode == ResultMode[ResultMode.ABS])) {
    newRobPose.x = robotAbsXYA.X
    newRobPose.y = robotAbsXYA.Y
    newRobPose.thetaZ = robotAbsXYA.A
    newRobPose.z = trainedRobotPose.z
    newRobPose.thetaY = trainedRobotPose.thetaY
    newRobPose.thetaX = trainedRobotPose.thetaX
    newRobPose.valid = 1
  } else if ((resMode == ResultMode.OFF) || (resMode == ResultMode[ResultMode.OFF])) {
    newRobPose.x = robotOffXYA.X
    newRobPose.y = robotOffXYA.Y
    newRobPose.z = 0.0
    newRobPose.thetaZ = robotOffXYA.A
    newRobPose.thetaY = 0.0
    newRobPose.thetaX = 0.0
    newRobPose.valid = 1
  } else if ((resMode == ResultMode.FRAME) || (resMode == ResultMode[ResultMode.FRAME])) {
    newRobPose.x = robotFrameXYA.X
    newRobPose.y = robotFrameXYA.Y
    newRobPose.z = 0.0
    newRobPose.thetaZ = robotFrameXYA.A
    newRobPose.thetaY = 0.0
    newRobPose.thetaX = 0.0
    newRobPose.valid = 1
    g_FrameCorrections[partID1].x = robotFrameXYA.X
    g_FrameCorrections[partID1].y = robotFrameXYA.Y
    g_FrameCorrections[partID1].thetaZ = robotFrameXYA.A
    g_FrameCorrections[partID1].valid = 1
  } else if ((resMode == ResultMode.PICKED) || (resMode == ResultMode[ResultMode.PICKED])) {
    newRobPose.x = robotPickedXYA.X
    newRobPose.y = robotPickedXYA.Y
    newRobPose.z = robPose.z
    newRobPose.thetaZ = robotPickedXYA.A
    newRobPose.thetaY = robPose.thetaY
    newRobPose.thetaX = robPose.thetaX
    newRobPose.valid = 1
  } else if ((resMode == ResultMode.GC) || (resMode == ResultMode[ResultMode.GC])) {
    let homeFromTargetTrained = new cogMath.cc2XformLinear()
    homeFromTargetTrained.setXform(trainResult['Home2DFromTarget2D'].xform)

    let homeFromTargetRuntime = new cogMath.cc2XformLinear()
    homeFromTargetRuntime.setXform(runtimeResult['Home2DFromTarget2D'].xform)

    let homeFromHandRuntime = new cogMath.cc2Rigid()
    homeFromHandRuntime.setXform(robPose.thetaZ, [robPose.x, robPose.y])

    let handFromTargetRuntime = homeFromHandRuntime.inverse().compose(homeFromTargetRuntime)

    let newHomeFromHandRuntime = homeFromTargetTrained.compose(handFromTargetRuntime.inverse())

    let gripCorrection = newHomeFromHandRuntime.inverse().compose(homeFromHandRuntime)

    newRobPose.x = gripCorrection.trans()[0]
    newRobPose.y = gripCorrection.trans()[1]
    newRobPose.z = 0
    newRobPose.thetaZ = gripCorrection.rotationInDegrees()
    newRobPose.thetaY = 0
    newRobPose.thetaX = 0
    newRobPose.valid = 1

    g_Parts[partID1].gripCorrection[gripperID] = newRobPose
  } else if ((resMode == ResultMode.GCP) || (resMode == ResultMode[ResultMode.GCP])) {

    let handFromTargetTrained = new cogMath.cc2XformLinear()
    handFromTargetTrained.setXform(trainResult['Robot2DFromTarget2D'].xform)

    let homeFromTargetRuntime = new cogMath.cc2XformLinear()
    homeFromTargetRuntime.setXform(runtimeResult['Home2DFromTarget2D'].xform)

    if (!g_Parts.hasOwnProperty(partID2)) {
      newRobPose.valid = ECodes.E_INVALID_PART_ID
      return newRobPose
    }

    let gc = g_Parts[partID2].gripCorrection[gripperID]
    if (gc.valid === 1) {
      let gripCorrection = new cogMath.cc2Rigid()
      gripCorrection.setXform(gc.thetaZ, [gc.x, gc.y])
      let newHomeFromHand = homeFromTargetRuntime.compose(handFromTargetTrained.inverse()).compose(gripCorrection.inverse())
      newRobPose.x = newHomeFromHand.trans()[0]
      newRobPose.y = newHomeFromHand.trans()[1]
      newRobPose.z = trainedRobotPose.z
      newRobPose.thetaZ = newHomeFromHand.rotationInDegrees()
      newRobPose.thetaY = trainedRobotPose.thetaY
      newRobPose.thetaX = trainedRobotPose.thetaX
      newRobPose.valid = 1
    } else {
      newRobPose.valid = ECodes.E_NO_GC_RESULT
    }
  } else {
    newRobPose.valid = ECodes.E_INVALID_ARGUMENT
  }

  if (newRobPose.valid > 0) {
    if (newRobPose.thetaZ > 180) {
      newRobPose.thetaZ-=360
    }
    if (newRobPose.thetaZ < -180) {
      newRobPose.thetaZ += 360
    }
  }

  return newRobPose
}
AS200.prototype.ComputeRobotPoses = function (shuttlingPoseIndex,
  robotPoseXYATrained,
  featurePositionsTrained,
  featurePositionsRuntime) {
  myLog('-> Compute Robot Pose ')

  let result = { "valid": ECodes.E_INTERNAL_ERROR }
  let absPosition = {}
  let offPosition = {}

  try {
    let robotPoseTrained = new RobotPose(robotPoseXYATrained.x, robotPoseXYATrained.y, 0, robotPoseXYATrained.thetaZ, 0, 0, robotPoseXYATrained.valid)
    let trainResult = ComputeTrained(shuttlingPoseIndex, robotPoseTrained, featurePositionsTrained)

    let runtimeResult = ComputeRuntime(shuttlingPoseIndex, featurePositionsRuntime)
    absPosition = ComputeAbsolut(shuttlingPoseIndex, trainResult, runtimeResult)

    offPosition = ComputeOffset(shuttlingPoseIndex, trainResult, absPosition)


    result['abs'] = absPosition
    result['off'] = offPosition
    result['valid'] = 1
  } catch (e) {
    myLog('Exception ComputeSeveralRobotPoses')
    myLog(e)
    result['abs'] = {}
    result['off'] = {}
    result['valid'] = ECodes.E_UNSPECIFIED
  }
  myLog('<- Compute Robot Pose ')
  return result
}
function ComputeOffset(shuttlingPoseIndex, trainResult, homeFromHandRT_u) {
  myLog('-> Compute Offset ')
  let calibXforms = g_Calibrations[shuttlingPoseIndex].results.Transforms

  // The offset position
  let tmpLinear = new cogMath.cc2XformLinear()
  let homeFromHandTrained = new cogMath.cc2Rigid()
  tmpLinear.setXform(trainResult['Home2DFromRobot2D'].xform)
  homeFromHandTrained = cogMath.lin2Rigid(tmpLinear)

  // Convert to uncorrected motion space
  var homeFromHandTrained_u = cogMath.convertToUncorrected(calibXforms, homeFromHandTrained)

  var robotOffXYA = {}
  robotOffXYA['theta'] = homeFromHandRT_u['theta'] - homeFromHandTrained_u.angleInDegrees()
  robotOffXYA['x'] = homeFromHandRT_u['x'] - homeFromHandTrained_u.trans()[0]
  robotOffXYA['y'] = homeFromHandRT_u['y'] - homeFromHandTrained_u.trans()[1]
  robotOffXYA['valid'] = 1
  myLog('<- Compute Offset ')
  return robotOffXYA
}
function ComputeAbsolut(shuttlingPoseIndex, trainResult, runtimeResult) {
  myLog('-> Compute Abs ')

  let calibXforms = g_Calibrations[shuttlingPoseIndex].results.Transforms
  // Step is trained, we are running the "XT" command
  var tmpLinear = new cogMath.cc2XformLinear()
  tmpLinear.setXform(trainResult['Robot2DFromTarget2D'].xform)
  var handFromTarget = cogMath.lin2Rigid(tmpLinear)

  var tmpLinear = new cogMath.cc2XformLinear()
  tmpLinear.setXform(runtimeResult['Home2DFromTarget2D'].xform)
  var homeFromTarget = cogMath.lin2Rigid(tmpLinear)
  var homeFromHandRT = homeFromTarget.compose(handFromTarget.inverse())
  var homeFromHandRT_u = cogMath.convertToUncorrected(calibXforms, homeFromHandRT)

  var robotAbsXYA = {}
  robotAbsXYA['theta'] = homeFromHandRT_u.angleInDegrees()
  robotAbsXYA['x'] = homeFromHandRT_u.trans()[0]
  robotAbsXYA['y'] = homeFromHandRT_u.trans()[1]
  robotAbsXYA['valid'] = 1
  myLog('<- Compute Abs ')
  return robotAbsXYA
}

function ComputeRuntime(shuttlingPosIndex, featurePositonsRuntime) {
  let result = ECodes.E_INTERNAL_ERROR;

  myLog('-> Compute Runtime ')
  let runtimeData = ComputePosMean(featurePositonsRuntime)
  let runtimeResult = {}
  var homeFromTarget = new cogMath.cc2Rigid()
  homeFromTarget.setXform(runtimeData.theta, [runtimeData.x, runtimeData.y])
  runtimeResult['Home2DFromTarget2D'] = homeFromTarget

  result = runtimeResult
  myLog('<- Compute Runtime ')

  return result
}

function ComputeTrained(shuttlingPosIndex, robotPoseTrained, featurePosTrained) {
  let result = ECodes.E_INTERNAL_ERROR;

  myLog('-> Compute Trained ')
  for (let tf = 0; tf < featurePosTrained.length; tf++) {
    if (featurePosTrained[tf].valid <= 0) {
      result = ECodes.E_TARGET_POSE_NOT_TRAINED
      return result
    }
  }
  if (robotPoseTrained.valid <= 0) {
    result = ECodes.E_ROBOT_POSE_NOT_TRAINED
    return result
  }
  let trainedData = ComputePosMean(featurePosTrained)
  let calibXforms = g_Calibrations[shuttlingPosIndex].results.Transforms
  let trainResult = {}

  let homeFromTarget = new cogMath.cc2Rigid()
  homeFromTarget.setXform(trainedData.theta, [trainedData.x, trainedData.y])
  trainResult['Home2DFromTarget2D'] = homeFromTarget

  var homeFromHand = new cogMath.cc2Rigid()
  homeFromHand.setXform(robotPoseTrained.thetaZ, [robotPoseTrained.x, robotPoseTrained.y])
  homeFromHand = cogMath.convertToCorrected(calibXforms, homeFromHand)
  trainResult['Home2DFromRobot2D'] = homeFromHand

  var tmpLinear = homeFromHand.inverse().compose(homeFromTarget)
  var handFromTarget = cogMath.lin2Rigid(tmpLinear)
  trainResult['Robot2DFromTarget2D'] = handFromTarget

  result = trainResult
  myLog('<- Compute Trained ')
  return result
}

function ComputePosMean(featurePositions) {
  let mean = { 'x': 0, 'y': 0, 'theta': 0 }

  let length = featurePositions.length
  for (let i = 0; i < length; i++) {
    mean.x += featurePositions[i].x
    mean.y += featurePositions[i].y
    mean.theta += featurePositions[i].theta
  }

  if (featurePositions.length > 1) {
    mean.x /= length
    mean.y /= length
    mean.theta /= length

    mean.theta = InSightFunctions.fnPointToPoint(featurePositions[0].x, featurePositions[0].y, featurePositions[1].x, featurePositions[1].y, 0).getAngle()
  }
  return mean
}

function ComputeAlignMode_3(partID1, partID2, gripperID, resMode, robPose) {
  let newRobPose = new RobotPose(0, 0, 0, 0, 0, 0, 0)

  if (!g_Parts.hasOwnProperty(partID1)) {
    newRobPose.valid = ECodes.E_INVALID_PART_ID
    return newRobPose
  }

  let trainedRobotPose = cloneObj(g_Parts[partID1]['trainedRobotPose'][gripperID]) // rtr
  if (trainedRobotPose.valid <= 0) {
    newRobPose.valid = ECodes.E_ROBOT_POSE_NOT_TRAINED
    return newRobPose
  }

  let runtimeFeatuers = g_Parts[partID1]['runtimeFeatures']
  let trainedFeatuers = g_Parts[partID1]['trainedFeatures']

  for (let tf = 0; tf < trainedFeatuers.length; tf++) {
    if (trainedFeatuers[tf][gripperID].valid <= 0) {
      newRobPose.valid = ECodes.E_TARGET_POSE_NOT_TRAINED
      return newRobPose
    }
  }

  let shuttlePose = g_Parts[partID1]['featuresInfos'][0]['shuttlePose']

  if ((runtimeFeatuers.length != trainedFeatuers.length) || (runtimeFeatuers.length > 2) || (runtimeFeatuers.length < 1)) {
    newRobPose.valid = ECodes.E_UNSPECIFIED
    return newRobPose
  }
  let length = runtimeFeatuers.length
  let sumRuntime = { 'x': 0, 'y': 0, 'theta': 0 } // trt
  let sumTrained = { 'x': 0, 'y': 0, 'theta': 0 } // ttr

  for (let i = 0; i < length; i++) {
    sumRuntime.x += runtimeFeatuers[i].x
    sumRuntime.y += runtimeFeatuers[i].y
    sumRuntime.theta += runtimeFeatuers[i].thetaInDegrees

    sumTrained.x += trainedFeatuers[i][gripperID].x
    sumTrained.y += trainedFeatuers[i][gripperID].y
    sumTrained.theta += trainedFeatuers[i][gripperID].thetaInDegrees
  }

  if (length > 1) {
    sumRuntime.x /= length
    sumRuntime.y /= length
    sumRuntime.theta /= length

    sumTrained.x /= length
    sumTrained.y /= length
    sumTrained.theta /= length

    sumRuntime.theta = InSightFunctions.fnPointToPoint(runtimeFeatuers[0].x, runtimeFeatuers[0].y, runtimeFeatuers[1].x, runtimeFeatuers[1].y, 0).getAngle()
    sumTrained.theta = InSightFunctions.fnPointToPoint(trainedFeatuers[0][gripperID].x, trainedFeatuers[0][gripperID].y, trainedFeatuers[1][gripperID].x, trainedFeatuers[1][gripperID].y, 0).getAngle()
  }

  let calibXforms = g_Calibrations[shuttlePose].results.Transforms

  let trainResult = {}
  let runtimeResult = {}
  // Train
  {
    let homeFromTarget = new cogMath.cc2Rigid()
    homeFromTarget.setXform(sumTrained.theta, [sumTrained.x, sumTrained.y])
    trainResult['Home2DFromTarget2D'] = homeFromTarget

    var homeFromHand = new cogMath.cc2Rigid()
    homeFromHand.setXform(trainedRobotPose.thetaZ, [trainedRobotPose.x, trainedRobotPose.y])
    homeFromHand = cogMath.convertToCorrected(calibXforms, homeFromHand)
    trainResult['Home2DFromRobot2D'] = homeFromHand

    var tmpLinear = homeFromHand.inverse().compose(homeFromTarget)
    var handFromTarget = cogMath.lin2Rigid(tmpLinear)
    trainResult['Robot2DFromTarget2D'] = handFromTarget
  }
  // Runtime
  {
    var homeFromTarget = new cogMath.cc2Rigid()
    homeFromTarget.setXform(sumRuntime.theta, [sumRuntime.x, sumRuntime.y])
    runtimeResult['Home2DFromTarget2D'] = homeFromTarget

    // Step is trained, we are running the "XT" command
    var tmpLinear = new cogMath.cc2XformLinear()
    tmpLinear.setXform(trainResult['Robot2DFromTarget2D'].xform)
    var handFromTarget = cogMath.lin2Rigid(tmpLinear)

    var homeFromHandRT = new cogMath.cc2Rigid()
    homeFromHandRT = homeFromTarget.compose(handFromTarget.inverse())

    var homeFromHandRT_u = cogMath.convertToUncorrected(calibXforms, homeFromHandRT)

    var robotAbsXYA = {}
    robotAbsXYA['A'] = homeFromHandRT_u.angleInDegrees()
    robotAbsXYA['X'] = homeFromHandRT_u.trans()[0]
    robotAbsXYA['Y'] = homeFromHandRT_u.trans()[1]

    // The offset position
    var homeFromHandTrained = new cogMath.cc2Rigid()
    tmpLinear.setXform(trainResult['Home2DFromRobot2D'].xform)
    homeFromHandTrained = cogMath.lin2Rigid(tmpLinear)

    // Convert to uncorrected motion space
    var homeFromHandTrained_u = cogMath.convertToUncorrected(calibXforms, homeFromHandTrained)

    var trainFromRT = new cogMath.cc2Rigid()
    trainFromRT = homeFromHandRT_u.compose(homeFromHandTrained_u.inverse())
  }

  let homeFromHandPocket_u = new cogMath.cc2Rigid()
  homeFromHandPocket_u.setXform(robPose.thetaZ, [robPose.x, robPose.y])
  homeFromHandPocket_u = cogMath.convertToUncorrected(calibXforms, homeFromHandPocket_u)

  let homeFromPocketFrameCorrected = new cogMath.cc2Rigid()
  let fc = g_Parts[partID1].frameCorrection
  if (fc.valid === 1) {
    let tmp = new cogMath.cc2Rigid()
    tmp.setXform(fc.thetaZ, [fc.x, fc.y])
    homeFromPocketFrameCorrected = tmp.compose(homeFromHandPocket_u)
  } else {
    newRobPose.valid = ECodes.E_PART_NO_VALID_FRAME_CORRECTION
    return newRobPose
  }

  if ((resMode == ResultMode.GCP) || (resMode == ResultMode[ResultMode.GCP])) {
    let handFromTargetTrained = new cogMath.cc2XformLinear()
    handFromTargetTrained.setXform(trainResult['Robot2DFromTarget2D'].xform)

    let homeFromTargetRuntime = new cogMath.cc2XformLinear()
    homeFromTargetRuntime.setXform(runtimeResult['Home2DFromTarget2D'].xform)
    if (!g_Parts.hasOwnProperty(partID2)) {
      newRobPose.valid = ECodes.E_INVALID_PART_ID
      return newRobPose
    }
    let gc = g_Parts[partID2].gripCorrection[gripperID]
    let gripCorrection = new cogMath.cc2Rigid()
    if (gc.valid === 1) {
      gripCorrection.setXform(gc.thetaZ, [gc.x, gc.y])
    } else {
      newRobPose.valid = ECodes.E_PART_NO_VALID_GRIP_CORRECTION
      return newRobPose
    }

    let newHomeFromHand = homeFromPocketFrameCorrected.compose(gripCorrection.inverse())
    newRobPose.x = newHomeFromHand.trans()[0]
    newRobPose.y = newHomeFromHand.trans()[1]
    newRobPose.z = trainedRobotPose.z
    newRobPose.thetaZ = newHomeFromHand.angleInDegrees()
    newRobPose.thetaY = trainedRobotPose.thetaY
    newRobPose.thetaX = trainedRobotPose.thetaX
    newRobPose.valid = 1
  } else
    if ((resMode == ResultMode.ABS) || (resMode == ResultMode[ResultMode.ABS])) {
      newRobPose.x = homeFromPocketFrameCorrected.trans()[0]
      newRobPose.y = homeFromPocketFrameCorrected.trans()[1]
      newRobPose.z = trainedRobotPose.z
      newRobPose.thetaZ = homeFromPocketFrameCorrected.angleInDegrees()
      newRobPose.thetaY = trainedRobotPose.thetaY
      newRobPose.thetaX = trainedRobotPose.thetaX
      newRobPose.valid = 1
    } else {
      newRobPose.valid = ECodes.E_INVALID_ARGUMENT
    }
  return newRobPose
}

function checkAllFeaturesOfPartTrained(partID, gripperID){
  let part = g_Parts[partID]
  let trainedFeatures = part['trainedFeatures']
  let numFeatures = trainedFeatures.length

  for(let f = 0; f<numFeatures;f++){
    if(trainedFeatures[f][gripperID]['valid'] == false){
      return false
    }
  }
  return true
}
function checkAllRuntimeFeaturesOfPartValid(partID){
  let part = g_Parts[partID]
  let runtimeFeatures = part['runtimeFeatures']
  let numFeatures = runtimeFeatures.length

  for(let f = 0; f<numFeatures;f++){
    if(runtimeFeatures[f]['valid'] == false){
      return false
    }
  }
  return true
}

function getFeatureIDFromCameraAndCameraFeatureID (camID, camFeatureID){
  
  let features = g_LooukupTables['features']
  for(let fID in features ){
    if(( features[fID]['CamFeatureID'] == camFeatureID ) && (features[fID]['CameraID'] == camID )){
      return parseInt(fID)
    }
  }
  return -1
}

function getShuttlingPosFromFeatureID (featureID) {
  let steps = g_LooukupTables['steps']

  for(let s in steps){
    if(steps[s]['FeatureIDs'].indexOf(featureID) >= 0 ){
      return parseInt(steps[s]['ShuttlingPoses'][0])
    }
  }
  return -1
}
function getExpSettingsFromFeatureID (featureID) {
  let steps = g_LooukupTables['steps']

  for(let s in steps){
    if(steps[s]['FeatureIDs'].indexOf(featureID) >= 0 ){
      return parseInt(steps[s]['ExpSettings'][0])
    }
  }
  return -1
}
function getFeatureIDsFromFeatureMask (featureMask){
  let featureIDs=[]
  for(var i = 0; i < 8; i++){
    let m = ((featureMask>>i)& 0x01)

    if(m == 1){
      featureIDs.push(i+1)
    }
  }
  return featureIDs
}
/**
 * 
 * */
function Part() {
  this.runtimeFeatures = []
  this.trainedFeatures = []
  this.featuresInfos = []
  this.trainedRobotPose = []
  this.gripCorrection = []
 
};

//* ***************************************************************************/
// Transformations
//* ***************************************************************************/

/**
 * Returns the transformed (to world) feature pos
 * @param {any} calibrations
 * @param {any} shuttlingPoseIndex
 * @param {any} cameraIsMoving
 * @param {any} partIsMoving
 * @param {any} feature
 * @param {any} robot
 */
function getTransformed(calibrations, shuttlingPoseIndex, cameraIsMoving, partIsMoving, feature, robot) {
  
  let calibration = calibrations[shuttlingPoseIndex]

  let retFeaturedTransformed = new Feature()
  if (calibration.calibration != null) {
    let heXf = new cogMath.cc2XformLinear()
    let taXf = new cogMath.cc2Rigid()

    taXf.setXform(feature.thetaInDegrees, [feature.x, feature.y])

    if ((cameraIsMoving == 0) && (partIsMoving == 0)) {
      heXf.setXform(calibration['results']['Transforms']['Home2DFromImage2D']['xform'])
      retFeaturedTransformed.valid = 1
    } else
      if ((cameraIsMoving == 1) && (partIsMoving == 0)) {
        let stageFromImage = new cogMath.cc2XformLinear()
        let homeFromStage = new cogMath.cc2Rigid()

        homeFromStage.setXform(robot.thetaZ, [robot.x, robot.y])
        homeFromStage = cogMath.convertToCorrected(calibration['results']['Transforms'], homeFromStage)
        stageFromImage.setXform(calibration['results']['Transforms']['Stage2DFromImage2D']['xform'])

        heXf.setXform(calibration['results']['Transforms']['Stage2DFromImage2D']['xform'])
        heXf = homeFromStage.compose(stageFromImage)

        retFeaturedTransformed.valid = 1
      } else
        if ((cameraIsMoving == 0) && (partIsMoving == 1)) {
          let homeFromHand = new cogMath.cc2Rigid()
          homeFromHand.setXform(robot.thetaZ, [robot.x, robot.y])
          homeFromHand = cogMath.convertToCorrected(calibration['results']['Transforms'], homeFromHand)

          let xf = new cogMath.cc2XformLinear()
          xf.setXform(calibration['results']['Transforms']['Home2DFromImage2D']['xform'])          

          heXf = homeFromHand.inverse().compose(xf)

      retFeaturedTransformed.valid = 1
    } else {
      tracer.addMessage('Combination of moving camera and moving part is not allowed')
    }

    if (retFeaturedTransformed.valid == 1) {
      let comp = heXf.compose(taXf)
      retFeaturedTransformed.x = comp.trans()[0]
      retFeaturedTransformed.y = comp.trans()[1]
      if (comp.hasOwnProperty('rotationInDegrees') === true) {
        retFeaturedTransformed.thetaInDegrees = comp.rotationInDegrees()
      } else {
        retFeaturedTransformed.thetaInDegrees = comp.angleInDegrees()
      }
    }
  } else {
    retFeaturedTransformed.valid = ECodes.E_NOT_CALIBRATED
    if (shuttlingPoseIndex == 2) {
      InSightFunctions.fnSetCellValue('HECalibration.2.ShowNotValid', 1)
    }
  }
  return retFeaturedTransformed
};

/**
 * Returns the world coordinates from the given camera point
 * @param {any} calibrations
 * @param {any} shuttlingPoseIndex
 * @param {any} cam_X
 * @param {any} cam_Y
 * @param {any} cam_Angle
 * @param {any} rob_X
 * @param {any} rob_Y
 * @param {any} rob_Theta
 */
function getWorldFromCam(calibrations, shuttlingPoseIndex, cam_X, cam_Y, cam_Angle, rob_X, rob_Y, rob_Theta) {
  let world = new Feature(0, 0, 0, 0)

  let calibration = calibrations[shuttlingPoseIndex]

  if (calibration.runstatus == 1) {
    var homeFromTarget = new cogMath.cc2XformLinear()
    var homeFromImage = new cogMath.cc2XformLinear()

    var camFromTarget = new cogMath.cc2XformLinear()
    var tmp = new cogMath.cc2Rigid()
    tmp.setXform(cam_Angle, [cam_X, cam_Y])
    camFromTarget.setXform(tmp.xform)

    if (calibration.calibration.isCameraMoving_ == true) {
      world.valid = ECodes.E_COMMAND_NOT_ALLOWED
    } else {
      homeFromImage.setXform(calibration.results.Transforms.Home2DFromCameraCenter2D.xform)
      homeFromTarget = homeFromImage.compose(camFromTarget)
    }

    world.x = homeFromTarget.trans()[0]
    world.y = homeFromTarget.trans()[1]
    world.thetaInDegrees = homeFromTarget.rotationInDegrees()

    world.valid = 1
  } else {
    world.valid = ECodes.E_NOT_CALIBRATED
    if (shuttlingPoseIndex == 2) {
      InSightFunctions.fnSetCellValue('HECalibration.2.ShowNotValid', 1)
    }
  }
  return world
};
/**
 * Returns the camera coordinates from the given world coordinates
 * @param {any} calibrations
 * @param {any} shuttlingPoseIndex
 * @param {any} world_X
 * @param {any} world_Y
 * @param {any} world_Angle
 */
function getCamFromWorld(calibrations, shuttlingPoseIndex, world_X, world_Y, world_Angle) {
  var point = new Feature(0, 0, 0, 0)

  let calibration = calibrations[shuttlingPoseIndex]

  if (calibration.runstatus == 1) {
    if (calibration.calibration['isCameraMoving_'] == 1) {
    } else {
      let homeFromCamera = calibration.results.Transforms.Home2DFromCameraCenter2D

      var taXf = new cogMath.cc2Rigid()
      taXf.setXform(world_Angle, [world_X, world_Y])

      var comp = homeFromCamera.inverse().compose(taXf)

      point.x = comp.trans()[0]
      point.y = comp.trans()[1]
      point.thetaInDegrees = comp.rotationInDegrees()
      point.valid = 1
    }
  } else {
    point.valid = ECodes.E_NOT_CALIBRATED
    if (shuttlingPoseIndex == 2) {
      InSightFunctions.fnSetCellValue('HECalibration.2.ShowNotValid', 1)
    }
  }
  return point
};
/**
 * Returns the world coordinates from the given image point
 * @param {any} calibrations
 * @param {any} shuttlingPoseIndex
 * @param {any} img_X
 * @param {any} img_Y
 * @param {any} img_Angle
 * @param {any} rob_X
 * @param {any} rob_Y
 * @param {any} rob_Theta
 */
function getWorldFromImage(calibrations, shuttlingPoseIndex, img_X, img_Y, img_Angle, rob_X, rob_Y, rob_Theta) {
  var world = new Feature(0, 0, 0, 0)

  let calibration = calibrations[shuttlingPoseIndex]
  if (calibration.runstatus == 1) {
    var homeFromTarget = new cogMath.cc2XformLinear()
    var homeFromImage = new cogMath.cc2XformLinear()

    var imageFromTarget = new cogMath.cc2XformLinear()
    var tmp = new cogMath.cc2Rigid()
    tmp.setXform(img_Angle, [img_X, img_Y])
    imageFromTarget.setXform(tmp.xform)

    if (calibration.calibration.isCameraMoving_ == true) {
      var stageFromImage = new cogMath.cc2XformLinear()
      stageFromImage.setXform(calibration.results.Transforms.Stage2DFromImage2D.xform)

      var homeFromHand = new cogMath.cc2XformLinear()
      var tmp2 = new cogMath.cc2Rigid()

      tmp2.setXform(rob_Theta, [rob_X * 1.0, rob_Y * 1.0])
      homeFromHand.setXform(tmp2.xform)

      homeFromTarget = homeFromHand.compose(stageFromImage).compose(imageFromTarget)
    } else {
      homeFromImage.setXform(calibration.results.Transforms.Home2DFromImage2D.xform)
      homeFromTarget = homeFromImage.compose(imageFromTarget)
    }

    world.x = homeFromTarget.trans()[0]
    world.y = homeFromTarget.trans()[1]
    world.thetaInDegrees = homeFromTarget.rotationInDegrees()
    world.valid = 1
  } else {
    world.valid = ECodes.E_NOT_CALIBRATED
    if (shuttlingPoseIndex == 2) {
      InSightFunctions.fnSetCellValue('HECalibration.2.ShowNotValid', 1)
    }
  }
  return world
};
/**
 * Converts world coordinates into image values.
 * At the moment moving camera are not implemented,
 * this is why rob_x, rob_y and rob_Theta are not used
 * @param {any} calibrations
 * @param {any} shuttlingPoseIndex
 * @param {any} world_X
 * @param {any} world_Y
 * @param {any} world_Angle
 * @param {any} rob_X
 * @param {any} rob_Y
 * @param {any} rob_Theta
 */
function getImageFromWorld(calibrations, shuttlingPoseIndex, world_X, world_Y, world_Angle, rob_X, rob_Y, rob_Theta) {
  var point = new Feature(0, 0, 0, 0)

  let calibration = calibrations[shuttlingPoseIndex]

  if (calibration.runstatus == 1) {
    if (calibration.calibration['isCameraMoving_'] == 1) {
      point.valid = ECodes.E_COMMAND_NOT_ALLOWED
    } else {
      var homeFromImage = calibration.results.Transforms.Home2DFromImage2D

      var taXf = new cogMath.cc2Rigid()
      taXf.setXform(world_Angle, [world_X, world_Y])

      var comp = homeFromImage.inverse().compose(taXf)

      point.x = comp.trans()[0]
      point.y = comp.trans()[1]
      point.thetaInDegrees = comp.rotationInDegrees()
      point.valid = 1
    }
  } else {
    point.valid = ECodes.E_NOT_CALIBRATED
    if (shuttlingPoseIndex == 2) {
      InSightFunctions.fnSetCellValue('HECalibration.2.ShowNotValid', 1)
    }
  }
  return point
}


/*****************************************************************************
 * Logger
 * The logger handles the logging for the HMI.
 * The object holds all messages and the type of the message
 * Each message type is shown different on the HMI
 * 0 -> dim gray normal message
 * 1 -> green
 * 2 -> yellow
 * 3 -> orange
 * 4 -> red
 * 5 -> gray     incoming command
 * ***************************************************************************/
function Logger() {
  this.messages = []
  this.newMessages = []
  this.logCounter = 0
};
/**
 * Adds a timestamp to the new message and adds it to the queue
 * @param {any} type 0 -> dim gray normal message
 * 1 -> green
 * 2 -> yellow
 * 3 -> orange
 * 4 -> red
 * 5 -> gray
 * @param {any} message the message
 */
Logger.prototype.addLogMessage = function (type, message) {
  
  let time = InSightFunctions.fnNow()
  let timeStr = InSightFunctions.fnGetGlock(time, "%H:%M:%S:%3N")
  let msg = InSightFunctions.fnStringf('%s %s', timeStr, message.slice(0, 200))// .toString() +": "+message;
  
  this.newMessages.push([type, msg])
}
/**
 * Cleares the log on the spread sheet
 * */
Logger.prototype.clearLog = function () {
  for (var i = 1; i <= MAX_LOGGING_ENTRIES; i++) {
    InSightFunctions.fnSetCellValue('Logging.MessageType_' + i.toString(), 0)
    InSightFunctions.fnSetCellValue('Logging.Message_' + i.toString(), '')    
  }
  //writeCellValue("Logging.FTP.Message","")
  this.messages = []
  this.logCounter = 0
  this._updateSheet()
}
/**
 * Writes all messages to the spread sheet
 * */
Logger.prototype.writeLogToSheet = function () {
  tracer.addMessage('-> Write Log ' + timeTracker.getElapsedTime())

  let newLength = this.newMessages.length
  let ftpMessageString = ''

  for (let n = 0; n < newLength; n++) {
    this.messages.splice(0, 0, this.newMessages[n])
    ftpMessageString = ftpMessageString + this.newMessages[n][1] +"\r\n"    
  }

  this.newMessages = []

  let logLength = this.messages.length

  if (logLength > MAX_LOGGING_ENTRIES) {
    this.messages.splice(60, logLength - 60)
  }
  logLength = this.messages.length

  for (var i = 0; i < logLength; i++) {
    InSightFunctions.fnSetCellValue('Logging.MessageType_' + (i + 1).toString(), this.messages[i][0])
    InSightFunctions.fnSetCellValue('Logging.Message_' + (i + 1).toString(), this.messages[i][1])
  }

  this._updateSheet()

  tracer.addMessage('<- Write Log ' + timeTracker.getElapsedTime())
  return ftpMessageString
}
/**
 * Start the update of the spread sheet and all counters
 * (Needed for the HMI)
 * */
Logger.prototype._updateSheet = function () {
  tracer.addMessage('-> Update Logger on the sheet ' + timeTracker.getElapsedTime())
  this.logCounter++
  let logLength = this.messages.length
  InSightFunctions.fnSetCellValue('Logging.Counter', this.logCounter)
  InSightFunctions.fnSetCellValue('Logging.Count', logLength)

  InSightFunctions.fnUpdateGui(1)

  tracer.addMessage('<- Update Logger on the sheet ' + timeTracker.getElapsedTime())
}
//* ***************************************************************************/
// functions
//* ***************************************************************************/
/**
 * This function adds a sub-"class" to an super class.
 * -> This is used to create something like a derived class in C#
 * -> The super class will be added to the sub-class's constructor
 * @param {any} Super
 * @param {any} Sub
 */
function inheritPseudoClass(Super, Sub) {
  Sub.prototype = Object_create(Super.prototype)
  Sub.prototype.constructor = Sub
};
/**
 * Creates the object to be added to the constructor 
 * of the sub-class.
 * @param {any} o
 */
function Object_create(o) {
  var F = function () { }
  F.prototype = o
  return new F()
};
/**
 * Checks if the received string is a valid command,
 * if yes it returns the command object
 * @param {any} commands list of all commands (master or slave)
 * @param {any} myIndex index if the camera
 * @param {any} cmdStr the received command string
 */
function getCommandObject(commands, myIndex, cmdStr) {
  var cmdObj = ECodes.E_UNKNOWN_COMMAND // 1 Valid / -1 Wrong number of arguments / -2 Wrong argument type / -3 Index out of range / -4 Unknown command
  var splittedCmdStr = cmdStr.toUpperCase().split(',')
  if (typeof commands[splittedCmdStr[0]] === 'function') {
    cmdObj = new (commands[splittedCmdStr[0]])(myIndex, cmdStr)
    let valid = cmdObj.isValid()
    if (valid != 1) {
      cmdObj = valid
    }
  }
  return cmdObj
};

function createDefaultFeaturesData() {
  let trainedFeatures = {}

  for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {

    trainedFeatures[f] = []
    for (let g = 0; g < MAX_GRIPPERS; g++) {
      trainedFeatures[f][g] = new Feature(0, 0, 0, 0)
    }
  }
  return trainedFeatures
}

function createDefaultRobotPosesData() {
  let trainedRobotPoses = {}

  for (let f = 1; f <= MAX_FEATURES_PER_CAMERA; f++) {
    trainedRobotPoses[f] = []
    for (let g = 0; g < MAX_GRIPPERS; g++) {
      trainedRobotPoses[f][g] = new RobotPose(0, 0, 0, 0, 0, 0, 0)
    }
  }
  return trainedRobotPoses
}
/**
 * Sets the acquisition settings for a specific inspection
 * @param {any} index ID of the inspection
 * @param {any} enabled is the inspection is enabled
 */
function setInspectionAcqSettings(index, enabled) {
  if (g_Inspections.hasOwnProperty(index)) {
    let settings = g_Inspections[index].acqSettings
    InSightFunctions.fnSetCellValue('InspectionAcquisitionSettings.ExposureTime', settings.exposure)
    InSightFunctions.fnSetCellValue('InspectionAcquisitionSettings.LightControl.Mode', settings.mode)
    InSightFunctions.fnSetCellValue('InspectionAcquisitionSettings.LightControl.Light_1', settings.light1)
    InSightFunctions.fnSetCellValue('InspectionAcquisitionSettings.LightControl.Light_2', settings.light2)
    InSightFunctions.fnSetCellValue('InspectionAcquisitionSettings.LightControl.Light_3', settings.light3)
    InSightFunctions.fnSetCellValue('InspectionAcquisitionSettings.LightControl.Light_4', settings.light4)
  }

  if (enabled) {
    if (checkTagNameAvailable('AcquisitionSettings.IndexSelector')) {
      writeCellValue('AcquisitionSettings.IndexSelector', 0)
      writeCellValue('AcquisitionSettings.TypeSelector', 1)
    } else {
      InSightFunctions.fnSetCellValue('AcquisitionSettings.Selector', 2)
    }
  }
}
/**
 * Selects the acquisition setting for old and new jobs.
 * First it checks if the new index selector cell is available (new jobs)
 * and if the index selector is available both (type and index selector) will be set 
 * to feature  and to the passed index, if not only the "old" selector will be set to the index
 * @param {any} index
 */
function setFeatureAcqSettings(index) {
  
    if (checkTagNameAvailable('AcquisitionSettings.IndexSelector')) {
      writeCellValue('AcquisitionSettings.TypeSelector', 0)
      writeCellValue('AcquisitionSettings.IndexSelector', index)
      
    } else {
      InSightFunctions.fnSetCellValue('AcquisitionSettings.Selector', index)
    }
}
/**
 * Checks if a tag name is in the spread sheet.
 * @param {any} tagName
 */
function checkTagNameAvailable(tagName) {
  var available = false
  try {
    let cell = InSightFunctions.fnGetCellName(tagName)
    available = true
  } catch (error) {
    tracer.addMessage('Tag <' + tagName + '> not found!')
  }
  return available
}
/**
 * Write a value to a special cell on the spread sheet.
 * @param {any} cellName
 * @param {any} value
 */
function writeCellValue(cellName, value) {
  try {
    InSightFunctions.fnSetCellValue(cellName, value)
  } catch (e) {
    myLog('Exception during writing CellValue <' + cellName + '> : <' + value + '>')
    myLog(e)
  }
}
/**
 * Read a value from the spread sheet
 * @param {any} cellName
 */
function readCellValue(cellName) {
  var retValue = null
  try {    
      retValue = InSightFunctions.fnGetCellValue(cellName)    
  } catch (e) {
    myLog('Exception during reading CellValue <' + cellName + '>')
    myLog(e)
  }
  return retValue
}
/**
 * Compares the content of two arrays.
 * The order of the elements inside the arrays are not important.
 * @param {any} arr_1
 * @param {any} arr_2
 */
function compareContentOfArrays(arr_1, arr_2) {
  if (arr_1.length !== arr_2.length) {
    return false
  }

  let length = arr_1.length
  for (let i = 0; i < length; i++) {
    if (arr_2.indexOf(arr_1[i]) < 0) {
      return false
    }
  }
  return true
}
/**
 * Checks if the mode is a valid mode and if the mode is in the allowed array.
 * In the case of an empty allowed array the complete defined modes are used
 * @param {any} mode one of the result mode as string or number (ABS, OFF,... 1,2,..)
 * @param {any} allowed an array off allowed modes ['ABS',3,4,'GC']
 */
function isValidResultMode(mode, allowed) {
  let result = false

  let index = getResultModeAsStr(mode)

  if (index) {
    if (allowed) {
      if ((allowed.indexOf(index) >= 0) || (allowed.indexOf(ResultMode[index]) >= 0)) {
        result = true
      }
    } else {
      result = true
    }
  }
  return result
}
/**
 * In a command is it possible to send the  result mode
 * as a string or as a number
 * -> ABS of 1
 * This function return always the string (ABS,..)
 * @param {any} mode
 */
function getResultModeAsStr(mode) {
  let result = mode
  let index = parseInt(mode)
  if (isNaN(index)) {
      return result
  }
  else {
        result = ResultMode[index]
  }
  return result
}

/**
 * Checks if the L-Check result mode is a valid mode and if the mode is in the allowed array.
 * In the case of an empty allowed array the complete defined modes are used
 * @param {any} mode one of the defined modes
 * @param {any} allowed an array of allowed modes [1,'DIFF']
 */
function isValidLCheckResultMode(mode, allowed) {
  let result = false
  let index = getLCheckResultModeAsStr(mode)

  if (index) {
    if (allowed) {
      if ((allowed.indexOf(index) >= 0) || (allowed.indexOf(ResultMode[index]) >= 0)) {
        result = true
      }
    } else {
      result = true
    }
  }
  return result
}
/**
 * In a command is it possible to send the L-Check result mode
 * as a string or as a number
 * -> ABS of 1
 * This function return always the string (ABS,..)
 * @param {any} mode
 */
function getLCheckResultModeAsStr(mode) {
  let result = mode
  let index = parseInt(mode)
    if (isNaN(index)) {
        return result
    }
    else {
        result = LCheckResultMode[index]
    }
  return result
}
/**
 * Checks if the coordinate system is a valid system and if the system is in the allowed array.
 * In the case of an empty allowed array the complete defined systems are used
 * @param {any} coordSys one of the defined systems
 * @param {any} allowed an array of allowed systems ['HOME2D','RAW2D']
 */
function isValidCoordinateSystem(coordSys, allowed) {
  let result = false
  let index = getCoordinateSystemAsStr(coordSys)

  if (index) {
    if (allowed) {
      if ((allowed.indexOf(index) >= 0) || (allowed.indexOf(ResultMode[index]) >= 0)) {
        result = true
      }
    } else {
      result = true
    }
  }
  return result
}
/**
 * In a command is it possible to send the coordinate system 
 * as a string or as a number
 * -> HOME2D of 1
 * This function return always the string (HOME2D,..)
 * @param {any} coordSys
 */
function getCoordinateSystemAsStr(coordSys) {
  let result = coordSys
  let index = parseInt(coordSys)
  if (isNaN(index)) {
      return result
  }
  else {
      result = CoordinateSystem[index]
  }

  return result
}

/**
 * Checks if the part id is in the part table
 * @param {any} id
 */
function isValidPartID(id) {
  return g_LooukupTables['parts'].hasOwnProperty(id)
}
/**
 * Checks if the step id is in the step table
 * @param {any} id
 */
function isValidStepID(id) {
  return g_LooukupTables['steps'].hasOwnProperty(id)
}
/**
 * Checks if the feature id is in the feature table
 * @param {any} id
 */
function isValidFeatureID(id) {
  return g_LooukupTables['features'].hasOwnProperty(id)
}
function isValidCameraID(id) {
  let result = false

  let index = g_LooukupTables.usedCameras.indexOf(id)
  if (index >= 0) {
    result = true
  }
  return result
}

/**
 * Creates two lines for the cross hair through the passed point with the passed angle
 * @param {any} row_0
 * @param {any} col_0
 * @param {any} angle in degrees
 */
function CrossHair(row_0, col_0, angle) {
  tracer.addMessage('-> CrossHair')
  // if the angle is 90 degrees
  if (Math.abs(angle % 90) < 1e-15) {
    angle = angle + 1e-15
  }

  let row_1 = row_0 + 100
  let col_1 = col_0 + (100 * Math.tan(Math.radians(angle)))

  let row_2 = row_0 + 100
  let col_2 = col_0 + (100 * Math.tan(Math.radians(angle + 90)))

  let line_1 = []
  let line_2 = []
  for (var i = 0; i < 4; i++) {
    var dist_1 = InSightFunctions.fnLineToLine(row_0, col_0, row_1, col_1, g_Border[i][0], g_Border[i][1], g_Border[i][2], g_Border[i][3], 0)
    if (dist_1.getDistance() == 0) {
      var row_intersect_1 = dist_1.getRow(0)
      var col_intersect_1 = dist_1.getCol(0)

      if ((row_intersect_1 >= 0) && (Math.floor(row_intersect_1) <= g_VRes) &&
        (col_intersect_1 >= 0) && (Math.floor(col_intersect_1) <= g_HRes)) {
        line_1.push([row_intersect_1, col_intersect_1])
      }
    }

    var dist_2 = InSightFunctions.fnLineToLine(row_0, col_0, row_2, col_2, g_Border[i][0], g_Border[i][1], g_Border[i][2], g_Border[i][3], 0)
    if (dist_2.getDistance() == 0) {
      var row_intersect_2 = dist_2.getRow(0)
      var col_intersect_2 = dist_2.getCol(0)

      if ((row_intersect_2 >= 0) && (Math.floor(row_intersect_2) <= g_VRes) &&
        (col_intersect_2 >= 0) && (Math.floor(col_intersect_2) <= g_HRes)) {
        line_2.push([row_intersect_2, col_intersect_2])
      }
    }
  }
  tracer.addMessage('<- CrossHair')
  return [[line_1[0][0], line_1[0][1], line_1[1][0], line_1[1][1]],
  [line_2[0][0], line_2[0][1], line_2[1][0], line_2[1][1]]]
};

/**
 * This function goes through all runtime features of a part
 * and checks if the feature is valid or not.
 * If the feature is not valid, the function returns 
 * the error code of the failed feature
 *  * @param {any} partID
 */
function checkPartRuntimeFeatures(partID) {

  if (g_Parts.hasOwnProperty(partID)) {
    let partRuntimeFeatures = g_Parts[partID]['runtimeFeatures']
    let length = partRuntimeFeatures.length
    for (let i = 0; i < length; i++) {
      if (partRuntimeFeatures[i]['valid'] <= 0) {
        return partRuntimeFeatures[i]['valid']
      }
    }
  } else {
    return ECodes.E_INVALID_PART_ID
  }
  return ECodes.E_NO_ERROR
}

function getLCheckJudgement(partID, AbsDist, DiffDist, RelDist) {
  let result = false;
  if (g_LooukupTables.parts === undefined) return false;
  if (g_LooukupTables.parts[partID].LCLimits === undefined) return false;
  if (g_LooukupTables.parts[partID].LCLimits.count < 2) return false;
  
  let low = g_LooukupTables.parts[partID].LCLimits[0];
  let high = g_LooukupTables.parts[partID].LCLimits[1];

  switch (g_LooukupTables.parts[partID].LCResultMode) {
    case 1: // Abs
      if (low <= AbsDist && AbsDist <= high) {
        result = true;
      }
      break;
    case 2: // Diff
      if (low <= DiffDist && DiffDist <= high) {
        result = true;
      }
      break;
    case 3: // Rel
      if (low <= RelDist && RelDist <= high) {
        result = true;
      }
      break;
    default:
      result = false;
  }

  return result;
}

/**
 * Computes the distance of feature 1 and feature 2 of a part
 * The function returns
 * - distance of the features
 * - absolute deviation between trained and runtime
 * - relative deviation between trained and runtime
 * 
 * @param {any} partID
 * @param {any} gripperID
 * 
 */
function getPartsFeatureDistance(partID, gripperID) {

  let result = { 'valid': false, 'ABS': 0, 'DIFF': 0, 'REL': 0, 'judge':false }

  if (g_Parts.hasOwnProperty(partID)) {
    
    if (g_Parts[partID].runtimeFeatures.length == 2) {
      let rtFeatures = []
      let ttFeatures = []
      for (let i = 0; i < 2; i++) {
        if (g_Parts[partID].runtimeFeatures[i]['valid'] <= 0) {
          result['valid'] = ECodes.E_PART_NOT_ALL_FEATURES_LOCATED
          return result
        }

        if (g_Parts[partID].trainedFeatures[i][gripperID]['valid'] <= 0) {
          result['valid'] = ECodes.E_FEATURE_NOT_TRAINED
          return result
        }

        rtFeatures.push(g_Parts[partID].runtimeFeatures[i])
        ttFeatures.push(g_Parts[partID].trainedFeatures[i][gripperID])
      }
      let rtDist = InSightFunctions.fnPointToPoint(rtFeatures[0].x, rtFeatures[0].y, rtFeatures[1].x, rtFeatures[1].y, 0).getDistance()
      let ttDist = InSightFunctions.fnPointToPoint(ttFeatures[0].x, ttFeatures[0].y, ttFeatures[1].x, ttFeatures[1].y, 0).getDistance()
      result['ABS'] = rtDist
      result['DIFF'] = ttDist - rtDist
      result['REL'] = ((ttDist - rtDist) * 100) / ttDist
      result['valid'] = true
      let jdg = getLCheckJudgement(partID, result['ABS'], result['DIFF'], result['REL'])
      result['judge'] = jdg
    } else {
      result['valid'] = ECodes.E_INVALID_PART_ID
    }
  }
  return result
}


/**
 * Creates a deep copy of an object
 * @param {any} obj
 */
function cloneObj(obj) {
  var clone = {}
  for (var i in obj) {
    if (obj[i] != null && typeof (obj[i]) === 'object') { clone[i] = cloneObj(obj[i]) } else { clone[i] = obj[i] }
  }
  return clone
}
/**
 * Checks if the passed string is a valid object
 * and returns the parsed object
 * @param {any} objString
 */
function checkAndGetObjectFromJsonString(objString) {
  let ret = null

  if (objString.length > 2) {
    if (typeof JSON.parse(objString) === 'object') {
      ret = JSON.parse(objString)
    }
  }
  return ret
};
/**
 * Converts degrees into radians
 * @param {any} degrees
 */
Math.radians = function (degrees) {
  return degrees * Math.PI / 180
}
/**
 * Converts radians into degrees
 * @param {any} radians
 */
Math.degrees = function (radians) {
  return radians * 180 / Math.PI
}
/**
 * Checks if the number between to limits. * 
 * @param {any} a limit one
 * @param {any} b limit two
 * @param {any} inclusive true = limits included, false = limits excluded
 */
Number.prototype.between = function (a, b, inclusive) {
  var min = Math.min(a, b)
  var max = Math.max(a, b)

  return inclusive ? this >= min && this <= max : this > min && this < max
}
/**
 * Checks if the delta is between 0 and max delta
 * @param {any} maxDelta max delta
 * @param {any} inclusive true = limits included, false = limits excluded
 */
Number.prototype.checkAngleDelta = function (maxDelta, inclusive) {
  var delta = Math.abs(this)

  if (delta > 180) { delta -= 360 }

  return Math.abs(delta).between(0, maxDelta, inclusive)
}


//* ***************************************************************************/
// "Singleton"
//* ***************************************************************************/

/**
 * Creates something like a singleton in C# and holds all 
 * shared objects .
 * With the add function it is possible to add new object.
 * The objects can be removed with the remove function.
 * To ask for an object you must use the get function.
 * */
var SharedObjects = (function () {
  // Instance stores a reference to the Singleton
  var instance
  function init() {
    // Singleton
    // Private methods and variables
    // function privateMethod () {
    //  console.log('I am private')
    // }

    var sharedObjects = {}
    // var privateVariable = "I am also private";

    return {
      // Public methods and variables
      addSharedObject: function (name, obj) {
        if (sharedObjects.hasOwnProperty(name)) {
          tracer.addMessage('Object-list contains such a Name!')
        } else {
          sharedObjects[name] = obj
        }
        return sharedObjects[name]
      },

      getSharedObject: function (name) {
        let obj = null
        if (sharedObjects.hasOwnProperty(name)) {
          obj = sharedObjects[name]
        }
        return obj
      },

      removeSharedObject: function (name) {
        if (sharedObjects.hasOwnProperty(name)) {
          delete sharedObjects[name]
        }
      }
      // publicProperty: "I am also public",
    }
  };

  return {
    // Get the Singleton instance if one exists
    // or create one if it doesn't
    getInstance: function () {
      if (!instance) {
        instance = init()
      }

      return instance
    }
  }
})()



//* ***************************************************************************/
// Make it visible 
//* ***************************************************************************/

module.exports.SharedObjects = SharedObjects
module.exports.ECodes = ECodes
module.exports.AS200 = AS200
module.exports.version = version

console.log('Loading AS200 done')
