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

#define __STRINGIFY_I(arg) #arg
#define __STRINGIFY(arg) __STRINGIFY_I(arg)
#define DEBUG_CODE(code) \
  std::cout<<"[Robot #"<<std::to_string(theRobotInfo.number)<<"] "<<__STRINGIFY(code)<<": "<<std::to_string(code)<<std::endl;

HRIControllerProvider::HRIControllerProvider(){}

void HRIControllerProvider::update(HRIController& controller)
{
    
    DEBUG_NUMB(PRINT_DEBUG, "HRIController update function\n\n\n");

    controller.ballCarrierDistanceThreshold = BALL_CARRIER_DISTANCE_THRESHOLD;
    controller.kickDistanceThreshold = KICK_DISTANCE_THRESHOLD;
    controller.reachPositionDistanceThreshold = REACH_POSITION_DISTANCE_THRESHOLD;

    controller.userPosition = userPosition;
    controller.userHeight = userHeight;


    //DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.taskQueue.empty()));
    
    controller.updateCurrentDestination = [&] (Vector2f destinationPose) -> void
    {
        controller.currentRobotDestination = destinationPose;
    };
  
    controller.scheduleInstructionsSpeech = [&] (bool initialSpeech, int taskID) -> void
    {
        if(controller.taskQueue.empty() || (controller.taskQueue.at(0).taskType != HRI::TaskType::InitialSpeech && controller.taskQueue.at(0).taskType != HRI::TaskType::InstructionsSpeech))
        {
            if(initialSpeech)
            {
                controller.taskQueue.insert(controller.taskQueue.begin(), InitialSpeechTask(taskID));
                controller.lastReceivedTaskID = taskID;
            }
            else
            {
                controller.taskQueue.insert(controller.taskQueue.begin(), InstructionsSpeechTask(taskID, theRobotPose.translation));
            }
        }
    };

    controller.updateCurrentBallDestination = [&] (Vector2f ballDestination) -> void
    {
        controller.currentBallDestination = ballDestination;
    };

    controller.getCurrentTask = [&] () -> Task
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.getCurrentTask");
        
        //If no task is requested
        if(controller.taskQueue.empty()) 
        {
            DEBUG_NUMB(PRINT_DEBUG, "Task queue empty");
            if(!controller.initialSpeechPerformed && PERFORM_INITIAL_SPEECH)
            {
                DEBUG_NUMB(PRINT_DEBUG, "Perform initial speech");
                controller.scheduleInstructionsSpeech(true, 0);
                controller.initialSpeechPerformed = true;
            }
            else
            {
                DEBUG_NUMB(PRINT_DEBUG, "Idle task");
                ::std::vector<Action> idleQueue = ::std::vector<Action>({IdleAction()});
                Task idleTask = Task(HRI::TaskType::None, -1, idleQueue, theRobotPose.translation);
                return idleTask; //SINGLETON;
            }
        }
        return controller.taskQueue.at(0);
    };

    controller.signalTaskCompleted = [&] (bool playSound) -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());
        DEBUG_NUMB(PRINT_DEBUG, "taskCompleted");
        controller.completedTasks.push_back(controller.taskQueue.at(0));
        if(playSound) SoundPlayer::play("TaskCompleted.wav");
        controller.nextTask();
    };

    controller.nextTask = [&] () -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.nextTask");
        if(!controller.taskQueue.empty())
        {
            DEBUG_NUMB(PRINT_DEBUG, "Task queue not empty");
            DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.taskQueue.at(0).taskID));
            controller.lastCompletedTaskID = controller.taskQueue.at(0).taskID;
            controller.taskQueue.erase(controller.taskQueue.begin());
            controller.currentAction = 0;
            DEBUG_NUMB(PRINT_DEBUG, std::to_string(controller.currentAction));
        }    
        //DEBUG_NUMB(PRINT_DEBUG, "Gone to next task");    
    };

    controller.getCurrentAction = [&] () -> Action
    {
        //DEBUG_NUMB(PRINT_DEBUG, "getCurrentAction");
        if(controller.taskQueue.empty() || controller.taskQueue.at(0).actionQueue.empty()) return IdleAction();
        return controller.taskQueue.at(0).actionQueue.at(controller.currentAction);
    };

    //If current action was completed, remove it from the task
    controller.nextAction = [&] () -> Action
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.nextAction");
        if(!controller.isTaskComplete())
        {
            //DEBUG_NUMB(PRINT_DEBUG, "task not complete: current action before "<<controller.currentAction);
            controller.currentAction+=1;
            //DEBUG_NUMB(PRINT_DEBUG, "task not complete: current action after "<<controller.currentAction);
        }
        controller.checkTaskCompleted(true);
        return controller.getCurrentAction();
    };

    controller.deleteSingleTask = [&] (int taskID) -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.deleteSingleTask");
        
        DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());

        
        
        if(controller.getCurrentTask().taskID == taskID) 
        {
            controller.nextTask();
        }
        else
        {
            Task* selectedTask = nullptr;
            int i=0;
            for(auto task : controller.taskQueue)
            {
                DEBUG_NUMB(PRINT_DEBUG, std::to_string(task.taskID));
                if(task.taskID == taskID) 
                {
                    DEBUG_NUMB(PRINT_DEBUG, "found task "+std::to_string(task.taskID));
                    selectedTask = &task;
                    break;
                }
                i++;
            }
            
            if(!selectedTask) return;
            
            controller.taskQueue.erase(controller.taskQueue.begin() + i);   
        }
    };

    controller.resetTaskQueue = [&] () -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.resetTaskQueue");
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
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.getCurrentActionType");
        DEBUG_CODE(controller.taskQueue.size());
        DEBUG_CODE(controller.currentAction);
        if(controller.taskQueue.empty() || controller.taskQueue.at(0).actionQueue.empty()) return HRI::ActionType::Idle;
        return controller.taskQueue.at(0).actionQueue.at(controller.currentAction).actionType;
    };

    controller.checkActionCompleted = [&] (bool condition) -> bool
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.checkActionCompleted");
        std::cout<<"taskID: "<<controller.getCurrentTask().taskID<<std::endl;
        if(condition)
        {
            controller.nextAction();
            return true;
        }
        return false;
    };

    controller.isTaskComplete = [&] () -> bool
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.isTaskComplete");
        if(controller.taskQueue.empty()) return false; //Idle task is never complete
        DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());
        DEBUG_NUMB(PRINT_DEBUG, "taskQueue not empty");
        DEBUG_NUMB(PRINT_DEBUG, "controller.taskQueue.at(0).taskSize: "<<controller.taskQueue.at(0).taskSize);
        DEBUG_NUMB(PRINT_DEBUG, "controller.currentAction: "<<controller.currentAction);
        return controller.currentAction >= controller.taskQueue.at(0).taskSize;
    };

    controller.checkTaskCompleted = [&] (bool playSound) -> bool
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.checkTaskCompleted");
        if(controller.isTaskComplete())
        {
            DEBUG_NUMB(PRINT_DEBUG, "Current tasks: "<<controller.tasksToString());
            DEBUG_NUMB(PRINT_DEBUG, "taskCompleted");
            controller.completedTasks.push_back(controller.taskQueue.at(0));

            if(playSound) SoundPlayer::play("TaskCompleted.wav");

            controller.nextTask();
            return true;
        }
        return false;
    };
  
    controller.addTask = [&] (Task task) -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.addTask");
        DEBUG_NUMB(PRINT_DEBUG, "Adding task: task.taskID: "<<std::to_string(task.taskID)<<", controller.lastReceivedTaskID: "<<std::to_string(controller.lastReceivedTaskID));
        //Only add newer tasks, that have a higher taskID
        if(task.taskID <= controller.lastReceivedTaskID) return;

        controller.taskQueue.push_back(task);
        controller.lastReceivedTaskID = task.taskID;
        DEBUG_NUMB(PRINT_DEBUG, "Task added");
    };
    
    controller.updateTasks = [&] (std::vector<Task> newTasks) -> void
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.updateTasks");
        DEBUG_NUMB(PRINT_DEBUG, "Updating tasks");
        for(auto task : newTasks) controller.addTask(task);
        DEBUG_NUMB(PRINT_DEBUG, "Tasks updated");
    };

    controller.actionToString = [&] (Action action) -> std::string
    {
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.actionToString");
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
        DEBUG_NUMB(PRINT_DEBUG, "\n\ncontroller.tasksToString");
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
                if(taskIndex>0 || actionIndex == controller.currentAction)
                    result<<controller.actionToString(action)<<std::string("\n");
                actionIndex++;
            }
            taskIndex++;
        }
        return result.str();
    };

    
    //Update loop

    controller.GRAPHICAL_DEBUG = (GRAPHICAL_DEBUG==1 ? true : false);

    Task currentTask = controller.getCurrentTask();

    if(controller.checkTaskCompleted(true))
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
                
                bool completed = controller.checkActionCompleted(
                    theLibCheck.distance(theRobotPose.translation, controller.currentRobotDestination)<controller.reachPositionDistanceThreshold
                );
                break;
            }
            case HRI::ActionType::ReachBall:
            {
                controller.updateCurrentDestination(globalBall);
                controller.updateCurrentBallDestination(globalBall);

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