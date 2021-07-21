/**
 * @file HRIController.h
 *
 * Declaration of the STREAMABLE struct HRIController to hold informations about the current state of the HRI routine
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

#include "Representations/Modeling/BallModel.h"

#include <iostream>

/*#define TASK(taskName)\
\
  STREAMABLE_WITH_BASE(taskName##Task, HRI::Task,\
  {\
    taskName##Task(int taskID);\ 
    taskName##Task(int taskID, std::vector<HRI::Action> actionQueue);\ 
\
    void addAction(HRI::Action action)\
    {\
      actionQueue.push_back(action);\ 
    }\
    ,\
    (std::vector<HRI::Action>) actionQueue,\
  });\
  inline taskName##Task(int taskID) : Task(HRI::TaskType::taskName, taskID) {};\ 
  inline taskName##Task(int taskID, std::vector<HRI::Action> actionQueue) : Task(HRI::TaskType::taskName, taskID) COMMA actionQueue(actionQueue) {};

TASK(Idle);*/
/*#define IDLE_ACTION(actionName, executeBody, isCompletedBody)\
class actionName##Action : public Action\
{\
  public:\
    actionName##Action(int actionID) : Action(actionID, HRI::ActionType::actionName) {}\
\
    void execute(HRIController& controller)\
    {\
    executeBody\
    setExecuted();\
    }\
\
    bool isCompleted() override\
    isCompletedBody\
\
    bool isIdle() override {return true;}\
};

#define ACTION(actionName, executeBody, isCompletedBody)\
class actionName##Action : public Action\
{\
  public:\
    actionName##Action(int actionID) : Action(actionID, HRI::ActionType::actionName) {}\
\
    void execute(HRIController& controller)\
    {\
    executeBody\
    setExecuted();\
    }\
\
    bool isCompleted() override\
    isCompletedBody\
\
};*/

namespace HRI
{
    ENUM(ActionType,
    {,
      Idle,
      ReachPosition,
      ReachBall,
      CarryBall,
      Kick,
      CarryAndKickToGoal,
      PerformInitialSpeech,
      PerformInstructionsSpeech,
    });

    ENUM(TaskType,
    {,
      None, //Only used when the task list is empty
      GoToPosition,
      //TurnToPosition,
      KickBallToPosition,
      CarryBallToPosition,
      ScoreGoalTask,
      //PointAtPosition,
      //PointAtBall,
      InitialSpeech,
      InstructionsSpeech,
    });

}

/* An action provides an execute method that modifies the HRIController state to execute the right behavior
  to complete the action */
STREAMABLE(Action,
{
  Action() = default;
  Action(HRI::ActionType actionType);
  Action(HRI::ActionType actionType, Vector2f target);
  
  /*Action& operator=(const Action& other)
  {
    return *this;
  }*/
  
  virtual ~Action() = default;
  
  void draw(bool graphicalDebug);
  ,
  (bool) completed,
  (HRI::ActionType) actionType,
  (Vector2f) target,
});
//CTOR for target-based actions 
inline Action::Action(HRI::ActionType actionType) : actionType(actionType), target(Vector2f(0,0)) 
{
  //std::cout<<"actionType: "<<TypeRegistry::getEnumName(actionType)<<std::endl;
  //Verify that the action is not target-based
  ASSERT(
    actionType!=HRI::ActionType::CarryBall 
    &&
    actionType!=HRI::ActionType::Kick 
    &&
    actionType!=HRI::ActionType::ReachPosition 
  );
};
//CTOR for actions not based on a target
inline Action::Action(HRI::ActionType actionType, Vector2f target) : actionType(actionType), target(target) 
{
  //std::cout<<"actionType: "<<TypeRegistry::getEnumName(actionType)<<std::endl;
  //Verify that the action is target-based
  ASSERT(
    actionType!=HRI::ActionType::Idle 
    &&
    actionType!=HRI::ActionType::ReachBall
    &&
    actionType!=HRI::ActionType::PerformInitialSpeech
    &&
    actionType!=HRI::ActionType::PerformInstructionsSpeech
  );
};

STREAMABLE_WITH_BASE(IdleAction, Action,
{
  IdleAction();
  ,
});
inline IdleAction::IdleAction() : Action(HRI::ActionType::Idle) {};

STREAMABLE(Task,
{     
  Task() = default; 
  Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue, Vector2f finalPosition);
  Task(HRI::TaskType taskType, int taskID);
  /*Task& operator=(const Task& other)
  {
    return *this;
  }*/

  ,
  (std::vector<Action>) actionQueue,

  (HRI::TaskType) taskType,
  (int) taskID,
  (int) taskSize,
  (Vector2f) finalPosition,
    
});
inline Task::Task(HRI::TaskType taskType, int taskID, std::vector<Action>& actionQueue, Vector2f finalPosition) : taskType(taskType), taskID(taskID), actionQueue(actionQueue), finalPosition(finalPosition), taskSize(actionQueue.size()) {};
inline Task::Task(HRI::TaskType taskType, int taskID) : taskType(taskType), taskID(taskID), taskSize(0) {};

/**
 * @struct HRIController
 * 
 * Struct containing state info about the HRI routine
 * 
 */
STREAMABLE(HRIController,
{
  /** Draws model on the field */
  void draw() const;

  FUNCTION(void(Vector2f destinationPose)) updateCurrentDestination;
  FUNCTION(void(Vector2f ballDestination)) updateCurrentBallDestination;
  FUNCTION(Task()) getCurrentTask;
  FUNCTION(void()) nextTask;
  FUNCTION(void()) resetTaskQueue;
  FUNCTION(HRI::ActionType()) getCurrentActionType;
  FUNCTION(bool(bool condition)) checkActionCompleted;
  FUNCTION(bool(bool playSound)) checkTaskCompleted;
  FUNCTION(void(Task)) addTask;
  FUNCTION(void(std::vector<Task>)) updateTasks;
  FUNCTION(std::string(Action action)) actionToString;
  FUNCTION(std::string()) tasksToString;
  FUNCTION(Action()) getCurrentAction;
  FUNCTION(Action()) nextAction;
  FUNCTION(bool()) isTaskComplete;
  FUNCTION(void(int taskID)) deleteSingleTask;

  FUNCTION(void(bool initialSpeech, int taskID)) scheduleInstructionsSpeech;
  FUNCTION(void()) scheduleIdleTask;

  FUNCTION(void(bool playSound)) signalTaskCompleted;

  HRIController();
  ,

  (bool) GRAPHICAL_DEBUG,

  (std::vector<Task>) completedTasks,
  (std::vector<Task>) taskQueue,
  (int)(0) currentAction,

  (Vector2f) currentRobotDestination,
  (Vector2f) currentBallDestination,

  (int)(-1) lastReceivedTaskID,
  (int)(-1) lastCompletedTaskID,

  (float) ballCarrierDistanceThreshold,
  (float) reachPositionDistanceThreshold,
  (float) kickDistanceThreshold,

  (Vector2f) userPosition,
  (float) userHeight,

  (bool)(false) initialSpeechPerformed,

});
inline HRIController::HRIController() : taskQueue(), completedTasks() {}
//TODO Expose function to update task queue