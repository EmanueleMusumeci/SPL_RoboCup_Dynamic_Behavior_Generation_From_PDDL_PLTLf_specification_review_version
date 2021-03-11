/**
 * @file Representations/BehaviorControl/BallCarrierModel.h
 *
 * Declaration of struct BallCarrierModel, that provides a model of the opponent goal
 * from the point of view of the striker, including obstacle coverage and
 * a score for the utility of each discretized targetable segment on the goal line
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/BehaviorControl/Libraries/LibPathPlanner.h"

/**
 * @struct BallCarrierModel
 *
 * This representation is used only for debugging purposes
 * 
 * Struct containing various modeling info useful to the ball carrier
 */


STREAMABLE(BallCarrierModel,
{

  /** Draws model on the field */
  void draw() const;
  
  STREAMABLE(Node,
  {
    Node() = default;
    Node(Vector2f center, float radius);
    ,
    (Vector2f) center,
    (float) radius,
  });

  STREAMABLE(Edge,
  {
    Edge() = default;
    Edge(BallCarrierModel::Node fromNode, BallCarrierModel::Node toNode);
    ,
    (BallCarrierModel::Node) fromNode,
    (BallCarrierModel::Node) toNode,
  });
  ,

  (bool) graphicalDebug,                              /** Toggle for the graphical debug in SimRobot */
  (float) obstacleAvoidanceArcAngleStep,              /** When computing a path around an obstacle, determines the arc increase around it */
  (std::vector<Edge>) obstacleAvoidancePlan,          /** Filled with a list of obstacles, ordered as they would be encountered along the path */
  (std::vector<Node>) ballPath,                       /** Filled with a list of waypoints for the path */
  (bool) isFallbackPath,                              /** Determines whether the current path is a "safe" path or a fallback one (a fake/unsafe one, because a safe one could not be found)*/
  (bool) isTargetOnGoal,                              /** Determines whether the current dynamic target is on the goal line*/
  (bool) isTargetInPenaltyArea,                       /** Determines whether the current dynamic target is in the penalty area*/
  (bool) isTargetAPass,                               /** Determines whether the current dynamic target is a pass target*/
  (Pose2f) dynamicTarget,                             /** Next immediate target for the ball along the path */
  //(Pose2f) dynamicApproachPoint,                    /** Entry point for the approach area, aligned with the ball and the dynamicTarget */
});

inline BallCarrierModel::Node::Node(Vector2f center, float radius) : center(center), radius(radius) {};
inline BallCarrierModel::Edge::Edge(BallCarrierModel::Node fromNode, BallCarrierModel::Node toNode) : fromNode(fromNode), toNode(toNode) {};