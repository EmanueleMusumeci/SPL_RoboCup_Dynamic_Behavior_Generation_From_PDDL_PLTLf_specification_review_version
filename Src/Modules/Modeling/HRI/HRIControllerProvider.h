/**
 * @file HRIControllerProvider.h
 *
 * This module keeps track of the state of execution of the Obstacle Avoidance HRI routine 
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/HRI/HRIController.h"
#include "Representations/Modeling/ExternalServerCommunicationController/ExternalServerCommunicationControl.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Representations/HRI/HRIController.h"

#include <iostream>
#include <ostream>

MODULE(HRIControllerProvider,
{,
    REQUIRES(LibCheck),
    REQUIRES(FieldDimensions),
    REQUIRES(BallSpecification),
    REQUIRES(GameInfo),
    REQUIRES(RobotInfo),
    REQUIRES(RobotPose),
    REQUIRES(BallModel),

    USES(BehaviorStatus),

    PROVIDES(HRIController),
    
    LOADS_PARAMETERS(
    {,
      //goalTarget constants
      (bool) GRAPHICAL_DEBUG,                                       /** Shows a graphical debug render in SimRobot */
      (bool) PRINT_DEBUG,
      
      (float) BALL_CARRIER_DISTANCE_THRESHOLD,                      /** Minimum distance of the ball from its destination to declare the CarryBall action completed */ 
      (float) KICK_DISTANCE_THRESHOLD,                              /** Minimum distance of the ball from its destination to declare the Kick action completed */ 
      (float) REACH_POSITION_DISTANCE_THRESHOLD,                    /** Minimum distance of the robot from its destination to declare the ReachPosition action completed */ 
    }),
});

/**
 * @class HRIControllerProvider
 * A module that provides the model of the opponent goal
 */
class HRIControllerProvider: public HRIControllerProviderBase
{
public:
  /** Constructor*/
  HRIControllerProvider();
  static Task GoToPositionTask(Vector2f position, int taskID);
  static Task CarryBallToPositionTask(Vector2f position, int taskID);
  static Task KickBallToPositionTask(Vector2f position, int taskID);
  static Task ScoreGoalTask(int taskID);

private:
  void update(HRIController& controller) override;
  bool checkCompleted(HRI::ActionType actionType);
};

inline Task HRIControllerProvider::GoToPositionTask(Vector2f position, int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachPosition, position));
  
  return Task(HRI::TaskType::GoToPosition, taskID, actionQueue, position);
}

inline Task HRIControllerProvider::CarryBallToPositionTask(Vector2f position, int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachBall));
  actionQueue.push_back(Action(HRI::ActionType::CarryBall, position));
  
  return Task(HRI::TaskType::CarryBallToPosition, taskID, actionQueue, position);
}

inline Task HRIControllerProvider::KickBallToPositionTask(Vector2f position, int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachBall));
  actionQueue.push_back(Action(HRI::ActionType::Kick, position));
  return Task(HRI::TaskType::KickBallToPosition, taskID, actionQueue, position);
}

inline Task HRIControllerProvider::ScoreGoalTask(int taskID)
{
  std::vector<Action> actionQueue;
  actionQueue.push_back(Action(HRI::ActionType::ReachBall));
  actionQueue.push_back(Action(HRI::ActionType::CarryAndKickToGoal));
  return Task(HRI::TaskType::ScoreGoalTask, taskID, actionQueue, Vector2f(0,0));
}