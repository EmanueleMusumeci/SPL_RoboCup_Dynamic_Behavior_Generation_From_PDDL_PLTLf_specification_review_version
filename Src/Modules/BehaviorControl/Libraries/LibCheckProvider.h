/**
 * @file LibCheckProvider.h
 *
 * Performs some checks of the behavior control module such as
 * - How often is a MotionRequest set?
 * - How often is a HeadMotionRequest set?
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"
//RECENTLY ADDED
//Goal targeting system
#include "Representations/Modeling/OpponentGoalModel.h"
#include "Representations/Configuration/BallSpecification.h"
//Striker potential field
#include "Representations/Modeling/NodePF.h"
#include "Representations/Modeling/StrikerPFModel.h"
//
#include "Representations/spqr_representations/RoleAndContext.h"

#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/FreeCorridors.h"
#include "Representations/spqr_representations/OurDefinitions.h"

#include "Tools/Module/Module.h"
#include <math.h>

MODULE(LibCheckProvider,
{,
  USES(ActivationGraph),
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(ObstacleModel),
  REQUIRES(FreeCorridors),
  //RECENTLY ADDED
  //Goal targeting system
  USES(OpponentGoalModel),
  USES(BallSpecification),
  //StrikerPF
  USES(StrikerPFModel),
  //
  USES(TeamActivationGraph),
  USES(TeamBehaviorStatus),
  USES(TeamData),
  USES(Role),
  USES(RoleAndContext),
  PROVIDES(LibCheck),
  LOADS_PARAMETERS(
  {,
    (std::vector<int>) notSetCheck,       /** Assert that a request has been set at least once */
    (std::vector<int>) multipleSetCheck,  /** Print a warning if an assert has not been set at least once */
    (bool) assertValidWalkRequest,        /** Asserts that there are no strange walk parameters */
  }),
});

class LibCheckProvider : public LibCheckProviderBase
{
private:
  int callCounters[LibCheck::numOfCheckedOutputs]; /**< The counters for different checks */
  bool setArmsInThisFrame[Arms::numOfArms]; /**< This arm was set in this frame */

  /**
   * Updates LibCheck
   * @param libCheck The representation provided
   */
  void update(LibCheck& libCheck) override;

  /** Resets all status information */
  void reset();

  /**
   * Checks whether a behavior part set all its outputs
   * @param activationGraph The activation graph of the behavior part
   * @param start The first output ID to be checked
   * @param end The first output ID not to be checked after \c start
   */
  void checkOutputs(const ActivationGraph& activationGraph, LibCheck::CheckedOutput start, LibCheck::CheckedOutput end) const;

  /**
   * Checks whether the motion request is valid
   * @param activationGraph The activation graph of the individual behavior
   * @param theMotionRequest The motion request to check for validity
   */
  void checkMotionRequest(const ActivationGraph& activationGraph, const MotionRequest& theMotionRequest) const;

  /** Increments one counter */
  void inc(LibCheck::CheckedOutput outputToCheck);

  /**
   * Serializes an activation graph to a string
   * @param activationGraph The activation graph to convert
   * @return The compressed string that represents the activation graph
   */
  std::string getActivationGraphString(const ActivationGraph& activationGraph) const;

  /**
   * Provides the Pose to reach in ready state for each robot
   * @return the target pose
   */
  Pose2f myReadyPosition() const;

  /**
   * Provides the distance between two pose2f
   * @param p1 the first point
   * @param p2 the second point
   * @return the distance between p1 and p2
   */
  float distance(Pose2f p1, Pose2f p2) const;

  /**
   * Provides the distance between two points (x1,y1) and (x2,y2)
   * @param x1 the first point
   * @param y1 the second point
   * @param x2 the first point
   * @param y2 the second point
   * @return the distance between p1 and p2
   */
  float distance(float x1, float y1, float x2, float y2) const;

  /** Provides a target, on the line between the robot and t, at a defined distance d
   * @param t the target to be refined (global)
   * @param d the desired distance
   * @return a target with distance d from the robot
   */
 Pose2f refineTarget(Pose2f t, float d);

 /** Returns the global y coord point we are looking at on the opponent groundline
  * @return Global y coord of the point we are looking at
  * **/
 float projectGazeOntoOpponentGroundline();

 /** Given a certain point in input proivdes its projection on the opponent ground line by the robot's perspective
  * @param x x global coordinate of point to be projected
  * @param y y global coordinate of point to be projected
  * @return Global y coord of the projected point
  * **/
 float projectPointOntoOpponentGroundline(float x, float y);

 /** Return the distance between my gaze projection and the obstacle
     left/right point onto groundline or 0 if the gaze is directly inside
     an obstacle
  * @param x x global coordinate of point to be projected
  * @param y y global coordinate of point to be projected
  * @return Global y coord of the projected point
  * **/
 float gazeToObstacleProjectionDistanceOntoOpponentGroundLine(Obstacle obs);

 /** Provides a float value representing a score for each FreeGoalTargetableArea Determined by computeFreeAreas
  * @param begin left limit of the free targetable area
  * @param end right limit of the free targetable area
  * @return value assigned after area evaluation
  * **/
 float areaValueHeuristic(float leftLimit, float rightLimit);

  /** Tells whether two segments are overlapping or not
  * @param l1 left limit of first segment
  * @param r1 right limit of the first segment
  * @param l2 left limit of the second segment
  * @param r2 right limit of the second segment
  * @return bool value
  * **/
 bool areOverlappingSegmentsOnYAxis (float l1, float r1, float l2, float r2);

 /** Provides a vector with the point of beginning and finish of goal areas free from opponent coverage 
  * @param myPose pose of the robot
  * @param opponents the opponent vector (global coordinates)
  * @return vector of free areas
  * **/
 std::vector<FreeGoalTargetableArea> computeFreeAreas (float minimumDiscretizedAreaSize);

 /** Based on a previous implementation by Emanuele Antonioni, provides the best point to shoot at inside the goal. If the opponent goal is completely occluded
  * returns the field center (exactly (0,0))
  * @param shootASAP If set to true has the robot will shoot to the nearest accessible point, located inside the nearest targetable area
  * @return the Vector2f of the position selected to shoot
  * **/
 Vector2f goalTarget (bool shootASAP);

 /** Computes the attractive field for the striker
  * @param goal The 2D position of the goal (that generates the attractive field)
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> compute_striker_attractive_PF(Vector2f goal, float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f, 
                                                  float Kr = 100.f, float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f);


 /** Computes the repulsive field for the striker
  * @param goal The 2D position of the goal (that generates the attractive field)
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> compute_striker_repulsive_PF(float RO = 1000.f, float Kap = 0.1f, float Kbp = 100.f, float Kr = 100.f, 
                                                  float TEAMMATE_CO = 500.f, float ETA = 1000.f, float GAMMA = 2.f);

 /** Initializes an empty PF
  * @param cell_size Side length of a discretized potential field cell
  * @return std::vector of NodePF with null potential
  * **/
 std::vector<NodePF> initialize_PF(float cell_size);

 /** Based on a previous implementation by Vincenzo Suriani, computes the an artificial potential field given a precomputed attractive and 
  * repulsive field respectively. Allows specifying a maximum cell update radius around the player. 
  * @param obstacles A vector of obstacles that generate the repulsive field
  * @param attractive_field An std::vector containing the precomputed attractive field, it has to be of the same size of repulsive_field
  * @param attractive_field An std::vector containing the precomputed repulsive field, it has to be of the same size of attractive_field
  * @param radius Maximum distance of the computed cells (cells that further away will not be updated). If left to 0.f, the whole field will be computed
  * @return std::vector of PFCell structures, containing various info about the potential field cell
  * **/
 std::vector<NodePF> computePF (std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field, float cell_size);


 int isTargetToPass;
 
 
 Pose2f glob2Rel(float x, float y);
 Pose2f rel2Glob(float x, float y);
 Vector2f getSupporterMarkPosition();
 Vector2f getSupportStrikerPosition();
 Vector2f getSupporterPosition();
 Vector2f getJollyPosition();
 Vector2f updateDefender();
 Vector2f updateSupporter();
 Vector2f updateGoalie();
 float distanceToLine(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2);
 bool obstacleExistsAroundPoint(Vector2f point);

 bool opponentOnOurField();
 std::tuple<int,int,Pose2f> strikerPassShare();
 bool isValueBalanced(float currentValue, float target, float bound);

 bool isGoalieInStartingPosition();
 bool isBallInKickAwayRange();
 bool isGoalieInKickAwayRange();
 bool isBallInArea();
 bool isGoalieInArea();
 bool isGoalieInAngle();
 float goalie_displacement;
 float angleToGoal;
 float angleToMyGoal;
 float penaltyAngle;
 float kickAngle;
 float correctionKickAngle;
 bool ballOutOnLeft;
 float radiansToDegree(float x);


 float angleToTarget(float x, float y);   // TODO This is to check orientation wrt to target x = 4500 y = 3000 to left and -3000 to right
 float norm(float x, float y);
 
 public: LibCheckProvider();
};
