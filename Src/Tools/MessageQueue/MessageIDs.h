/**
 * @file MessageIDs.h
 *
 * Declaration of ids for debug messages.
 *
 * @author Martin Lötzsch
 */

#pragma once

#include "Tools/Streams/Enum.h"

/**
 * IDs for debug messages
 *
 * To distinguish debug messages, they all have an id.
 */
ENUM(MessageID,
{,
  undefined,
  idFrameBegin,
  idFrameFinished,

  idActivationGraph,
  idAlternativeRobotPoseHypothesis,
  idAnnotation,
  idAudioData,
  idBallModel,
  idBallPercept,
  idBallSpots,
  idBehaviorStatus,
  idBodyContour,
  idCameraImage,
  idCameraInfo,
  idCameraMatrix,
  idCirclePercept,
  idFallDownState,
  idFieldBoundary,
  idFieldColors,
  idFieldCoverage,
  idFieldFeatureOverview,
  idFrameInfo,
  idFsrSensorData,
  idGameInfo,
  idGetUpEngineOutput,
  idGetUpEngineOutputLog,
  idGlobalOptions,
  idGroundTruthOdometryData,
  idGroundTruthWorldState,
  idImageCoordinateSystem,
  idInertialData,
  idInertialSensorData,
  idJointAngles,
  idJointCalibration,
  idJointLimits,
  idJointRequest,
  idJointSensorData,
  idJPEGImage,
  idKeyStates,
  idLabelImage,
  idLinesPercept,
  idMotionInfo,
  idMotionRequest,
  idObstacleModel,
  idObstaclesFieldPercept,
  idObstaclesImagePercept,
  idOdometer,
  idOdometryData,
  idOpponentTeamInfo,
  idOwnTeamInfo,
  idPenaltyMarkPercept,
  idRobotDimensions,
  idRobotHealth,
  idRobotInfo,
  idRobotPose,
  idSelfLocalizationHypotheses,
  idSideConfidence,
  idStopwatch,
  idSystemSensorData,
  idTeamActivationGraph,
  idTeamBallModel,
  idTeamBehaviorStatus,
  idTeamData,
  idTeamPlayersModel,
  idTeamTalk,
  idThumbnail,
  idWalkGenerator,
  idWalkGeneratorData,
  idWalkingEngineOutput,
  idWalkLearner,
  idWhistle,
  numOfDataMessageIDs, /**< everything below this does not belong into log files */

  // infrastructure
  idRobot = numOfDataMessageIDs,
  idConsole,
  idDebugDataChangeRequest,
  idDebugDataResponse,
  idDebugDrawing,
  idDebugDrawing3D,
  idDebugImage,
  idDebugRequest,
  idDebugResponse,
  idDrawingManager,
  idDrawingManager3D,
  idLogResponse,
  idModuleRequest,
  idModuleTable,
  idMotionNet,
  idPlot,
  idRobotname,
  idText,
  idTypeInfo,
  idTypeInfoRequest,
  idColorCalibration,

  //spqr
  idUtilityShare,
  idPassShare,
  idRoleAndContext,
  idShareMessage,
});
