/**
 * @file HRIControllerProvider.cpp
 *
 * This module keeps track of the state of execution of the Obstacle Avoidance HRI routine 
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "HRIControllerProvider.h"

#define DEBUG_NUMB(print_debug, message) \
  if(print_debug) std::cout<<"[Robot #"<<theRobotInfo.number<<"] " << message << std::endl; 

HRIControllerProvider::HRIControllerProvider(){}

void HRIControllerProvider::update(HRIController& controller)
{
    
    controller.ballCarrierDistanceThreshold = BALL_CARRIER_DISTANCE_THRESHOLD;
    controller.kickDistanceThreshold = KICK_DISTANCE_THRESHOLD;
    controller.reachPositionDistanceThreshold = REACH_POSITION_DISTANCE_THRESHOLD;


    //Lambda functions
    
    //Task creation
    /*controller.GoToPositionTask = [&] (Vector2f position, int taskID) -> Task
    {
        return HRIControllerProvider::GoToPositionTask(position, taskID);
    };

    controller.CarryBallToPositionTask = [&] (Vector2f position, int taskID) -> Task
    {
        return HRIControllerProvider::CarryBallToPositionTask(position, taskID);
    };

    controller.KickBallToPositionTask = [&] (Vector2f position, int taskID) -> Task
    {
        return HRIControllerProvider::KickBallToPositionTask(position, taskID);
    };

    controller.ScoreGoalTask = [&] (int taskID) -> Task
    {
        return HRIControllerProvider::ScoreGoalTask(taskID);
    };*/



    DEBUG_NUMB(PRINT_DEBUG, "HRIController update function");
    //DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.taskQueue.empty()));
    
    controller.updateCurrentDestination = [&] (Vector2f destinationPose) -> void
    {
        controller.currentRobotDestination = destinationPose;
    };
  
    controller.updateCurrentBallDestination = [&] (Vector2f ballDestination) -> void
    {
        controller.currentBallDestination = ballDestination;
    };

    controller.getCurrentTask = [&] () -> Task
    {
        DEBUG_NUMB(PRINT_DEBUG, "Get current task");
        if(controller.taskQueue.empty()) 
        {
            DEBUG_NUMB(PRINT_DEBUG, "Task queue empty");
            ::std::vector<Action> idleQueue = ::std::vector<Action>({IdleAction()});
            Task idleTask = Task(HRI::TaskType::None, -1, idleQueue, theRobotPose.translation);
            return idleTask; //SINGLETON;
        }
        return controller.taskQueue.at(0);
    };

    controller.nextTask = [&] () -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "Going to next task");
        if(!controller.taskQueue.empty())
        {
            DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.taskQueue.at(0).taskID));
            controller.lastCompletedTaskID = controller.taskQueue.at(0).taskID;
            controller.taskQueue.erase(controller.taskQueue.begin());
            controller.currentAction = 0;
        }    
        DEBUG_NUMB(PRINT_DEBUG, "Gone to next task");    
    };

    controller.getCurrentAction = [&] () -> Action
    {
        DEBUG_NUMB(PRINT_DEBUG, "getCurrentAction");
        if(controller.taskQueue.empty() || controller.taskQueue.at(0).actionQueue.empty()) return IdleAction();
        return controller.taskQueue.at(0).actionQueue.at(controller.currentAction);
    };

    //If current action was completed, remove it from the task
    controller.nextAction = [&] () -> Action
    {
        DEBUG_NUMB(PRINT_DEBUG, "nextAction");
        if(!controller.isTaskComplete())
        {
            DEBUG_NUMB(PRINT_DEBUG, "task not complete: current action before "<<controller.currentAction);
            controller.currentAction+=1;
            DEBUG_NUMB(PRINT_DEBUG, "task not complete: current action after "<<controller.currentAction);
        }
        controller.checkTaskCompleted();
        return controller.getCurrentAction();
    };

    controller.resetTaskQueue = [&] () -> void
    {
        int maxTaskID = -1;
        for(auto task : controller.taskQueue)
        {
            maxTaskID = std::max(task.taskID, maxTaskID);
        }
        controller.taskQueue.clear();
        controller.lastReceivedTaskID = maxTaskID + 1;
        controller.lastCompletedTaskID = maxTaskID + 1;
        controller.currentAction = 0;
    };
    
    controller.getCurrentActionType = [&] () -> HRI::ActionType
    {
        if(controller.taskQueue.empty() || controller.taskQueue.at(0).actionQueue.empty()) return HRI::ActionType::Idle;
        return controller.taskQueue.at(0).actionQueue.at(controller.currentAction).actionType;
    };

    controller.checkActionCompleted = [&] (bool condition) -> bool
    {
        if(condition)
        {
            controller.nextAction();
            return true;
        }
        return false;
    };

    controller.isTaskComplete = [&] () -> bool
    {
        if(controller.taskQueue.empty()) return false; //Idle task is never complete
        DEBUG_NUMB(PRINT_DEBUG, "controller.taskQueue.at(0).taskSize: "<<controller.taskQueue.at(0).taskSize);
        DEBUG_NUMB(PRINT_DEBUG, "controller.currentAction: "<<controller.currentAction);
        return controller.currentAction >= controller.taskQueue.at(0).taskSize;
    };

    controller.checkTaskCompleted = [&] () -> bool
    {
        //DEBUG_NUMB(PRINT_DEBUG, "checkTaskCompleted");
        if(controller.isTaskComplete())
        {
            DEBUG_NUMB(PRINT_DEBUG, "taskCompleted");
            controller.completedTasks.push_back(controller.taskQueue.at(0));
            controller.nextTask();
            return true;
        }
        return false;
    };
  
    controller.addTask = [&] (Task task) -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "Adding task: task.taskID: "<<std::to_string(task.taskID)<<", controller.lastReceivedTaskID: "<<std::to_string(controller.lastReceivedTaskID));
        //Only add newer tasks, that have a higher taskID
        if(task.taskID <= controller.lastReceivedTaskID) return;

        controller.taskQueue.push_back(task);
        controller.lastReceivedTaskID = task.taskID;
        DEBUG_NUMB(PRINT_DEBUG, "Task added");
    };
    
    controller.updateTasks = [&] (std::vector<Task> newTasks) -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "Updating tasks");
        for(auto task : newTasks) controller.addTask(task);
        DEBUG_NUMB(PRINT_DEBUG, "Tasks updated");
    };

    controller.actionToString = [&] (Action action) -> std::string
    {
        std::stringstream result;
        result<<"----Action: "<<TypeRegistry::getEnumName(action.actionType);
        switch(action.actionType)
        {
            case HRI::ActionType::Idle:
            {
                break;
            }
            case HRI::ActionType::ReachPosition:
            {
                result<<" (location: ("<<action.target.x()<<", "<<action.target.y()<<"))";
                break;
            }
            case HRI::ActionType::ReachBall:
            {
                Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
                result<<" ("<<globalBall.x()<<", "<<globalBall.y()<<")";
                break;
            }
            case HRI::ActionType::CarryBall:
            {
                result<<" (ballDestination: ("<<action.target.x()<<", "<<action.target.y()<<")";
                break;
            }
            case HRI::ActionType::Kick:
            {
                Vector2f goalTarget = theLibCheck.goalTarget(false, true);
                result<<" (ballDestination: ("<<goalTarget.x()<<", "<<goalTarget.y()<<")";
                break;
            }
            case HRI::ActionType::CarryAndKickToGoal:
            {
                Vector2f goalTarget = theLibCheck.goalTarget(false, true);
                result<<" (goalTarget: ("<<goalTarget.x()<<", "<<goalTarget.y()<<")";
                break;
            }
        }
        return result.str();
    };

    controller.tasksToString = [&] () -> std::string
    {
        std::stringstream result;
        DEBUG_NUMB(PRINT_DEBUG, "controller.taskQueue.size(): "<<controller.taskQueue.size());
        int taskIndex = 0;
        for(auto task : controller.taskQueue)
        {
            result<<std::string("Task ID: ")<<std::to_string(task.taskID)<<std::string("\n");
            result<<std::string("--Task: ")<<std::string(TypeRegistry::getEnumName(task.taskType))<<std::string("\n");
            int actionIndex = 0;
            for(auto action : task.actionQueue)
            {
                if(taskIndex>0 || actionIndex >= controller.currentAction)
                    result<<controller.actionToString(action)<<std::string("\n");
                actionIndex++;
            }
            taskIndex++;
        }
        return result.str();
    };

    DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());
    
    //Update loop

    controller.GRAPHICAL_DEBUG = (GRAPHICAL_DEBUG==1 ? true : false);


    Task currentTask = controller.getCurrentTask();

    if(controller.checkTaskCompleted())
    {
        return;
    }
    else
    {
        Action currentAction = controller.getCurrentAction();

        Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;

        switch(currentAction.actionType)
        {
            case HRI::ActionType::Idle:
            {
                controller.updateCurrentDestination(theRobotPose.translation);
                controller.updateCurrentBallDestination(globalBall);
                
                break;
            }
            case HRI::ActionType::ReachPosition:
            {
                controller.updateCurrentDestination(currentAction.target);
                controller.updateCurrentBallDestination(globalBall);
                
                /*
                DEBUG_NUMB(PRINT_DEBUG, "Current robot destination: ("<<controller.currentRobotDestination.x()<<", "<<controller.currentRobotDestination.y()<<")");
                DEBUG_NUMB(PRINT_DEBUG, "Current robot position: ("<<theRobotPose.translation.x()<<", "<<theRobotPose.translation.y()<<")");
                DEBUG_NUMB(PRINT_DEBUG, "Current ball destination: ("<<controller.currentBallDestination.x()<<", "<<controller.currentBallDestination.y()<<")");
                DEBUG_NUMB(PRINT_DEBUG, "theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination): "<<theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination));
                DEBUG_NUMB(PRINT_DEBUG, "reachPositionDistanceThreshold: "<<controller.reachPositionDistanceThreshold);
                */

                bool completed = controller.checkActionCompleted(
                    theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination)<controller.reachPositionDistanceThreshold
                );
                /*
                DEBUG_NUMB(PRINT_DEBUG, "Current action: "<<controller.currentAction);
                DEBUG_NUMB(PRINT_DEBUG, "Completed: "<<std::to_string(completed));
                */
                break;
            }
            case HRI::ActionType::ReachBall:
            {
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(globalBall);

                
                DEBUG_NUMB(PRINT_DEBUG, "Current robot destination: ("<<controller.currentRobotDestination.x()<<", "<<controller.currentRobotDestination.y()<<")");
                DEBUG_NUMB(PRINT_DEBUG, "Current robot position: ("<<theRobotPose.translation.x()<<", "<<theRobotPose.translation.y()<<")");
                DEBUG_NUMB(PRINT_DEBUG, "Current ball destination: ("<<controller.currentBallDestination.x()<<", "<<controller.currentBallDestination.y()<<")");
                DEBUG_NUMB(PRINT_DEBUG, "theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination): "<<theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination));
                DEBUG_NUMB(PRINT_DEBUG, "reachPositionDistanceThreshold: "<<controller.reachPositionDistanceThreshold);
                

                controller.checkActionCompleted(
                    theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination)<controller.reachPositionDistanceThreshold
                );
                break;
            }
            case HRI::ActionType::CarryBall:
            {
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(currentAction.target);
                
                controller.checkActionCompleted(
                    theLibCheck.distance(globalBall, controller.currentBallDestination)<controller.ballCarrierDistanceThreshold
                );
                break;
            }
            case HRI::ActionType::Kick:
            {
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(currentAction.target);
                
                controller.checkActionCompleted(
                    theLibCheck.distance(globalBall, controller.currentBallDestination)<controller.kickDistanceThreshold
                );
                break;
            }
            case HRI::ActionType::CarryAndKickToGoal:
            {
                Vector2f goalTarget = theLibCheck.goalTarget(false, true);
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(goalTarget);
                
                controller.checkActionCompleted(
                    theLibCheck.distance(globalBall, controller.currentBallDestination)<controller.kickDistanceThreshold
                );
                break;
            }
        }
    }
    
}


MAKE_MODULE(HRIControllerProvider, modeling)