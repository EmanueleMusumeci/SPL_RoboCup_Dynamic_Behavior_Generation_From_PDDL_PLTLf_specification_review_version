/**
 * @file LibCheck.h
 *
 * This file defines a representation that checks some behavior control properties
 *
 * @author Daniel Krause
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/FreeGoalTargetableArea.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/Modeling/NodePF.h"
#include "Tools/Function.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Representations/Modeling/ObstacleModel.h"

STREAMABLE(LibCheck,
{
  ENUM(CheckedOutput,
  {,
    motionRequest,
    headMotionRequest,
    activity,
    passTarget,
    firstTeamCheckedOutput,
    teamActivity = firstTeamCheckedOutput,
    timeToReachBall,
    teammateRoles,
    role,
  });

  /** Increments one counter */
  FUNCTION(void(LibCheck::CheckedOutput outputToCheck)) inc;

  /** Indicates that an arm has been set */
  FUNCTION(void(Arms::Arm arm)) setArm;

  /** Checks whether an arm has been set */
  FUNCTION(bool(Arms::Arm arm)) wasSetArm;

  /** Performs checks for the individual behavior */
  FUNCTION(void(const MotionRequest& theMotionRequest)) performCheck;
  /** Provides the ready position for each robot **/
  FUNCTION(Pose2f()) myReadyPosition;

  /** Provides the distance between 2 Pose2f **/
  FUNCTION(float(Pose2f p1, Pose2f p2)) distance;

  /** Provides a target, on the line between the robot and t, at a defined distance d **/
  FUNCTION(Pose2f(Pose2f t, float d)) refineTarget;

 /** Returns the global y coord point we are looking at on the opponent groundline*/
  FUNCTION(float()) projectGazeOntoOpponentGroundline;

 /** Given a certain point in input proivdes its projection on the opponent ground line by the robot's perspective*/
  FUNCTION(float(float x, float y)) projectPointOntoOpponentGroundline;

/**Return the distance between my gaze projection and the obstacle
     left/right point onto groundline or 0 if the gaze is directly inside
     an obstacle **/
  FUNCTION(float(Obstacle obs)) gazeToObstacleProjectionDistanceOntoOpponentGroundLine;

  /** Provides a float value representing a score for each FreeGoalTargetableArea Determined by computeFreeAreas **/
  FUNCTION(float(float leftLimit, float rightLimit, float poles_weight, float opponents_weight, float teammates_weight)) areaValueHeuristic;

  /** Tells whether two segments are overlapping or not **/
  FUNCTION(bool(float l1, float r1, float l2, float r2)) areOverlappingSegmentsOnYAxis;

  /** Provides a vector with the point of beginning and finish of goal areas free from opponent coverage **/
  FUNCTION(std::vector<FreeGoalTargetableArea>(float minimumDiscretizedAreaSize)) computeFreeAreas;

  /** Provides the better point to shoot at inside the goal 
  - shootASAP: if you're near the goal, shoot in the spot nearest to where you're looking at ("As Soon As Possible"), else use the heuristic to decide
  - forceHeuristic: always use the heuristic to decide where to shoot **/
  FUNCTION(Vector2f(bool shootASAP, bool forceHeuristic)) goalTarget;

  FUNCTION(float(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax)) mapToInterval;

  /** Performs checks for the team behavior */
  FUNCTION(void()) performTeamCheck;

  FUNCTION(bool(float currentValue, float target, float bound)) isValueBalanced;
  FUNCTION(float(float x, float y)) angleToTarget;
  FUNCTION(float(float x, float y)) norm;

  FUNCTION(float()) defenderDynamicDistance;
  FUNCTION(Pose2f()) nearestTemmate;


   // passing the ball
  FUNCTION(bool(std::vector<Obstacle>, Vector2f& target, const Vector2f& translation)) findPassingLineAux;
  FUNCTION(Vector2f(Vector2f target, std::vector<Vector2f> mates)) findPassingLine;
  FUNCTION(Pose2f()) poseToReceive;
  FUNCTION(Pose2f()) poseToPass;
  FUNCTION(bool(Pose2f targetPose, Pose2f shootingPose,std::vector<Obstacle> opponents)) canPass;

  FUNCTION(Pose2f(float x, float y)) glob2Rel;
  FUNCTION(Pose2f(float theta, float x, float y)) glob2RelWithAngle;
  FUNCTION(float(float x))radiansToDegree;

  FUNCTION(Pose2f(float x, float y)) rel2Glob;
  FUNCTION(Vector2f()) getSupporterMarkPosition;
  FUNCTION(Vector2f()) getSupportStrikerPosition;
  FUNCTION(Vector2f()) getSupporterPosition;
  FUNCTION(Vector2f()) getJollyPosition;
  FUNCTION(bool()) opponentOnOurField;
  FUNCTION(std::tuple<int,int,Pose2f>()) strikerPassShare;
  FUNCTION(float(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2)) distanceToLine;
  FUNCTION(bool(Vector2f point)) obstacleExistsAroundPoint;
  FUNCTION(float()) defenderDynamicY;


  FUNCTION(float(Vector2f p1, Vector2f p2)) angleBetweenPoints;

  FUNCTION(Vector3f(Pose2f targetPoint1, Pose2f targetPoint2, Pose2f feedbackPoint1, Pose2f feedbackPoint2)) getError;
  FUNCTION(bool()) C2OwnField;
  FUNCTION(bool()) C2PassingArea;
  FUNCTION(bool()) C2ReceiverArea;
  FUNCTION(int()) C2WhichCorridor;
  FUNCTION(Pose2f(int role)) C2EvaluateTarget;
  FUNCTION(Pose2f(Pose2f target)) C2EvaluateApproach;
  FUNCTION(float(Pose2f p1, Pose2f p2, bool inverse)) C2AngleBetween;
  FUNCTION(float()) C2AngleToTarget;
  FUNCTION(float()) C2AngleToTarget_bis;


  FUNCTION(bool(int)) strikerPassCommonConditions,


  (bool) isGoalieInStartingPosition,
  (bool) isGoalieInAngle,
  (bool) isBallInKickAwayRange,
  (bool) isGoalieInKickAwayRange,
  (bool) isBallInArea,
  (bool) isGoalieInArea,

  (Vector2f) goaliePosition,
  (Vector2f) defenderPosition,
  (Vector2f) supporterPosition,
  (Vector2f) jollyPosition,

  (float) angleForDefender,
  (float) angleForSupporter,
  (float) angleForJolly,
  (int) isTargetToPass,

  (int) timeSinceBallWasSeen,
  (float) goalie_displacement,
  (float) angleToGoal,
  (float) angleToBall,
  (float) angleToMyGoal,
  (float) penaltyAngle,
  (float) kickAngle,
  (float) correctionKickAngle,
  (bool) ballOutOnLeft,
});
