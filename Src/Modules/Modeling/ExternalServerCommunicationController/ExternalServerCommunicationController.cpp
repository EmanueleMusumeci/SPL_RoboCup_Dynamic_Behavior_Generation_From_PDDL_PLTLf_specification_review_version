#include "ExternalServerCommunicationController.h"
#include "Modules/Modeling/HRI/HRIControllerProvider.h"
#include "Tools/Modeling/Obstacle.h"
#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#define IP_ADDRESS "127.0.0.1"

#define CONTROL_DEVICE_COMMUNICATION_READ_PORT_BASE 65001
#define CONTROL_DEVICE_COMMUNICATION_WRITE_PORT_BASE 65101

#define DEBUG_NUMB(print_debug, message) \
  if(print_debug) std::cout<<"[Robot #"<<theRobotInfo.number<<"] " << message << std::endl; \

#define BUFFER_SIZE 1024

#define ACTION_QUEUE_TAG_STRING std::string("taskQueue")
#define LAST_TASK_ID_REQUEST_STRING std::string("lastTaskID?")
#define LAST_TASK_QUEUE_REQUEST_STRING std::string("lastTaskQueue?")
#define RESET_TASKS_STRING std::string("resetTasks")

//Macro used to determine if a string starts with another string
#define string_startswith(string, cmp_string) \
    string.rfind(cmp_string, 0) == 0

/*
#define __STRINGIFY__(taskType) taskType
#define CREATE_TASK(taskType, position, taskID) HRIControllerProvider::__STRINGIFY__(##taskType)##Task(position, taskID)
#define CORRECT_TASK_SELECTOR(taskVar, position, taskID, taskQueue) \
switch(##taskVar##)\
{\
    FOREACH_ENUM(HRI::TaskType, task)\
    {\
        case HRI::TaskType::task:\
        {\
            taskQueue.push_back(CREATE_TASK(task, position, taskID));\
        }\
    }\
}\
*/

//TODO Create an external NetworkProtocol class to customize network protocol
//TODO Automatize using factories, based on the provided NetworkProtocol instance

ExternalServerCommunicationController::ExternalServerCommunicationController(){

    int write_port_number = CONTROL_DEVICE_COMMUNICATION_WRITE_PORT_BASE + theRobotInfo.number-1;
    this->udp_write_socket.setTarget(IP_ADDRESS, write_port_number);
    this->udp_write_socket.setBlocking(false);
    DEBUG_NUMB(true, "Write socket set on address: "<<IP_ADDRESS<<":"<<write_port_number);

    int read_port_number = CONTROL_DEVICE_COMMUNICATION_READ_PORT_BASE + theRobotInfo.number-1;
    this->udp_read_socket.bind(IP_ADDRESS, read_port_number);
    this->udp_read_socket.setBlocking(false);
    DEBUG_NUMB(true, "Read socket bound on address: "<<IP_ADDRESS<<":"<<read_port_number);
    
    this->cycles_since_last_keepalive_check = 0; 
    this->client_alive = false;
    this->awaiting_keepalive_response = false;
    
    this->cycles_since_robot_pose_update = 0;
    this->cycles_since_ball_update = 0;
    this->cycles_since_obstacles_update = 0; 
}

std::string ExternalServerCommunicationController::keepalive_check_string(){
    std::string ret_string;
    ret_string.append("uthere?");
    return ret_string;
}

std::string ExternalServerCommunicationController::last_task_id_string(){
    std::string ret_string;
    ret_string.append("lastTaskID,");
    ret_string.append(std::to_string(theHRIController.lastReceivedTaskID));
    ret_string.append(",");
    ret_string.append(std::to_string(theHRIController.lastCompletedTaskID));
    return ret_string;
}

std::string ExternalServerCommunicationController::last_task_queue_string(){
    std::string ret_string;
    ret_string.append("lastTaskQueue;");
    ret_string.append("lastTaskID,");
    ret_string.append(std::to_string(theHRIController.lastReceivedTaskID));
    ret_string.append(",");
    ret_string.append(std::to_string(theHRIController.lastCompletedTaskID));
    for(auto task : theHRIController.taskQueue)
    {
        ret_string.append(";");
        switch(task.taskType)
        {
            case HRI::TaskType::GoToPosition:
            {
                ret_string.append("GoToPosition,"+std::to_string(task.taskID)+","+std::to_string(task.finalPosition.x())+","+std::to_string(task.finalPosition.y()));
                break;
            }
            case HRI::TaskType::CarryBallToPosition:
            {
                ret_string.append("CarryBallToPosition,"+std::to_string(task.taskID)+","+std::to_string(task.finalPosition.x())+","+std::to_string(task.finalPosition.y()));
                break;
            }
            case HRI::TaskType::KickBallToPosition:
            {
                ret_string.append("KickBallToPosition,"+std::to_string(task.taskID)+","+std::to_string(task.finalPosition.x())+","+std::to_string(task.finalPosition.y()));
                break;
            }
            case HRI::TaskType::ScoreGoalTask:
            {
                ret_string.append("ScoreGoal,"+std::to_string(task.taskID));
                break;
            }
        }
    }
        
    return ret_string;
}

std::string ExternalServerCommunicationController::robot_pose_to_sendable_string(){

    std::string ret_string;
    ret_string.append("robot_pose,");
    ret_string.append(std::to_string(theRobotPose.rotation));
    ret_string.append(",");
    ret_string.append(std::to_string(theRobotPose.translation.x()));
    ret_string.append(",");
    ret_string.append(std::to_string(theRobotPose.translation.y()));
    return ret_string;
    
}

std::string ExternalServerCommunicationController::ball_position_to_sendable_string(){

    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
    std::string ret_string;
    ret_string.append("ball_position,");
    ret_string.append(std::to_string(globalBall.x()));
    ret_string.append(",");
    ret_string.append(std::to_string(globalBall.y()));
    return ret_string;
    
}

std::string ExternalServerCommunicationController::obstacles_to_sendable_string(){

    std::string ret_string;
    std::string obs_string;

    int obs_number = 0;
    
    ret_string.append("obstacles");
    
    for(auto obs : theObstacleModel.obstacles){
        obs_number++;
        if(obs.type == Obstacle::opponent){
            Vector2f globalObstacle = theLibCheck.rel2Glob(obs.center.x(), obs.center.y()).translation;
            ret_string.append(";");
            ret_string.append(std::to_string(globalObstacle.x()));
            ret_string.append(",");
            ret_string.append(std::to_string(globalObstacle.y()));
        }
    }
    std::cout<<"Obstacles: "<<ret_string<<std::endl;
    return ret_string;
}

void ExternalServerCommunicationController::send_data_string(std::string str, bool prefix_timestamp, bool prefix_robot_number)
{
    std::string header;
    if(prefix_timestamp)
    {
        header.append("timestamp,");
        header.append(std::to_string(Time::getCurrentSystemTime()));
        header.append(std::string("."));
    }
    if(prefix_robot_number)
    {
        header.append("robot_number,");
        header.append(std::to_string(theRobotInfo.number));
    }
    header.append("|");

    str = header + str;
    const char *s_str = str.c_str();
    this->udp_write_socket.write(s_str, str.length());    
}

std::vector<std::string> ExternalServerCommunicationController::getTokens(std::string& str, std::string delimiter)
{
    std::vector<std::string> tokens;    

    int delimPosition = 0;
    while ((delimPosition = str.find(delimiter)) != std::string::npos) {
        tokens.push_back(str.substr(0, delimPosition));
        str.erase(0, delimPosition + delimiter.length());
    }
    tokens.push_back(str);

    return tokens;
}

void ExternalServerCommunicationController::handleMessage(std::string message, std::vector<Task>& currentTaskQueue)
{
    DEBUG_NUMB(PRINT_DEBUG,"Handling message: "<<message);
    DEBUG_NUMB(PRINT_DEBUG,"Message length:"<<std::to_string(message.length()));
    
    if(string_startswith(message, ACTION_QUEUE_TAG_STRING))
    {
        std::cout<<message<<std::endl;
        
        std::vector<std::string> taskTokens = getTokens(message, std::string("|"));
        
        ASSERT(taskTokens.size() == 2);
        taskTokens = getTokens(taskTokens[1], std::string(";"));
        DEBUG_NUMB(PRINT_DEBUG, taskTokens.size());

        //For every token in sliced vector excluding first token (message tag)
        for(auto taskToken : std::vector<std::string>(taskTokens.begin(), taskTokens.end())) 
        {
            std::cout<<taskToken<<std::endl;

            std::vector<std::string> taskParameters = getTokens(taskToken, std::string(","));
            
            //ASSERT(taskParameters.size()==2 || taskParameters.size()==4);
            
            //Parse task type
            //ASSERT(static_cast<HRI::TaskType>(TypeRegistry::getEnumValue(typeid(HRI::TaskType).name(), taskParameters[0])) != -1);
            if(static_cast<HRI::TaskType>(TypeRegistry::getEnumValue(typeid(HRI::TaskType).name(), taskParameters[0])) == -1)
            {
                std::cout<<"Unknown TaskType: "<<std::string(taskParameters[0])<<std::endl;
                return;
            }

            DEBUG_NUMB(PRINT_DEBUG, taskParameters.size());
            HRI::TaskType taskType = static_cast<HRI::TaskType>(TypeRegistry::getEnumValue(typeid(HRI::TaskType).name(), taskParameters[0]));
            
            //Based on task type, create the task instance and add it to the local task queue
            if(taskParameters.size()==4)
            {

                float coordX = std::stof(taskParameters[2]);
                float coordY = std::stof(taskParameters[3]);
                Vector2f position = Vector2f(coordX, coordY);

                //Parse taskID
                std::cout<<taskParameters[3]<<std::endl;
                int taskID = std::stoi(taskParameters[1]);

                switch(taskType)
                {
                    case HRI::TaskType::GoToPosition:
                    {
                        currentTaskQueue.push_back(HRIControllerProvider::GoToPositionTask(position, taskID));
                        break;
                    }
                    case HRI::TaskType::CarryBallToPosition:
                    {
                        currentTaskQueue.push_back(HRIControllerProvider::CarryBallToPositionTask(position, taskID));
                        break;
                    }
                    case HRI::TaskType::KickBallToPosition:
                    {
                        currentTaskQueue.push_back(HRIControllerProvider::KickBallToPositionTask(position, taskID));
                        break;
                    }
                    default:
                    {
                        std::cout << "ERROR: task "<<taskParameters[0]<<" was not recognized"<<std::endl;
                    }
                }
                DEBUG_NUMB(PRINT_DEBUG, currentTaskQueue.size());
            }
            else if(taskParameters.size()==2)
            {
                //Parse taskID
                int taskID = std::stoi(taskParameters[1]);

                switch(taskType)
                {
                    case HRI::TaskType::ScoreGoalTask:
                    {
                        currentTaskQueue.push_back(HRIControllerProvider::ScoreGoalTask(taskID));
                    }
                    default:
                    {
                        std::cout << "ERROR: task "<<taskParameters[0]<<" was not recognized"<<std::endl;
                        return;
                    }
                }
                
            }
            else
            {
                std::cout<<"WRONG MESSAGE STRUCTURE: "<<message<<std::endl;
            }
        }
    }
    if(string_startswith(message, LAST_TASK_ID_REQUEST_STRING))
    {
        send_data_string(last_task_id_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
    }
    if(string_startswith(message, LAST_TASK_QUEUE_REQUEST_STRING))
    {
        send_data_string(last_task_queue_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
    }
    if(string_startswith(message, RESET_TASKS_STRING))
    {
        theHRIController.resetTaskQueue();
    }
}


//TODO tell the device which actions are completed -> simply send the lastCompletedTaskID
//TODO receive actions from the device -> continuously receive the queued action (the device will deal with deleting completed actions from the queue)
//Automatically add interactions
void ExternalServerCommunicationController::update(ExternalServerCommunicationControl& externalServerCommunicationControl) {


    externalServerCommunicationControl.getNewTasks = [&] (long lastTaskID) -> std::vector<Task>
    {
        std::vector<Task> newTasks;
        for(auto task : externalServerCommunicationControl.currentTaskQueue)
        {
            if(task.taskID > lastTaskID) newTasks.push_back(task);
        }

        return newTasks;
    };

    /*externalServerCommunicationControl.currentTaskQueue = std::vector<Task>({
                                                                            HRIControllerProvider::GoToPositionTask(Vector2f(1000.f, 1000.f), 0),
                                                                            HRIControllerProvider::CarryBallToPositionTask(Vector2f(-2000.f, -2000.f), 1), 
                                                                            HRIControllerProvider::KickBallToPositionTask(Vector2f(-4500.f, 0.f), 2)
                                                                            });*/
    /*externalServerCommunicationControl.currentTaskQueue = std::vector<Task>({
                                                                            HRIControllerProvider::ScoreGoalTask(0)
                                                                            });*/
    theHRIController.updateTasks(externalServerCommunicationControl.currentTaskQueue);
    externalServerCommunicationControl.currentTaskQueue.clear();

    std::string recv_string;
    
    bool received = false;
    
    char* buffer = (char*) malloc(sizeof(char) * BUFFER_SIZE);

    //DEBUG_NUMB(PRINT_DEBUG,"HELLO");
    int bt = this->udp_read_socket.read(buffer, sizeof(char) * BUFFER_SIZE);

    if(bt != -1){
        received = true;

        recv_string += std::string(buffer);
        //std::cout<<"Data received"<<std::endl;
    }
    //DEBUG_NUMB(PRINT_DEBUG,"HELLO");

    free(buffer);
    

    if(received)
    {
        DEBUG_NUMB(PRINT_DEBUG,"Received string: "<<recv_string);
    }
    else
    {
        DEBUG_NUMB(PRINT_DEBUG,"Nothing received");
    }
    
    if(PERFORM_KEEPALIVE_CHECK)
    {
        //DEBUG_NUMB(PRINT_DEBUG,"this->cycles_since_last_keepalive_check: "<<this->cycles_since_last_keepalive_check);
        if(!this->client_alive || this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
        {
            if(this->awaiting_keepalive_response)
            {
                //DEBUG_NUMB(PRINT_DEBUG,"Checking keepalive response");
                if(received)
                {
                    //DEBUG_NUMB(PRINT_DEBUG,"Received keepalive response");
                    this->awaiting_keepalive_response = false;
                    this->client_alive = true;
                    this->cycles_since_last_keepalive_check = 1;
                }
                //To send another keepalive request after a while
                else if(this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
                {
                    this->awaiting_keepalive_response = false;
                }
            }
            else if(this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
            {
                this->client_alive = false;
                //DEBUG_NUMB(PRINT_DEBUG,"Checking if client is alive");
                send_data_string(keepalive_check_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
                this->awaiting_keepalive_response = true;
            }
        }
        //If waiting for a keepalive response, return until one is received
        if(!this->client_alive) return;

        this->cycles_since_last_keepalive_check++;

        //Ignore keepalive responses
        if(recv_string == "yeah") return;

        //DEBUG_NUMB(PRINT_DEBUG,"After keepalive");
    }

    //DEBUG_NUMB(PRINT_DEBUG,"B");
    
    if(this->cycles_since_robot_pose_update % ROBOT_POSE_UPDATE_FREQUENCY == 0){
        send_data_string(robot_pose_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
        this->cycles_since_robot_pose_update = 0;
    }
    this->cycles_since_robot_pose_update++;

    //DEBUG_NUMB(PRINT_DEBUG,"C");
    
    if(this->cycles_since_ball_update % BALL_POSITION_UPDATE_FREQUENCY == 0){
        send_data_string(ball_position_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
        this->cycles_since_ball_update = 0;
    }
    this->cycles_since_ball_update++;


    //DEBUG_NUMB(PRINT_DEBUG,"D");

    if(this->cycles_since_obstacles_update % OBSTACLES_UPDATE_FREQUENCY == 0){
        send_data_string(obstacles_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
        this->cycles_since_obstacles_update = 0;
    }
    this->cycles_since_obstacles_update++;
    
    //DEBUG_NUMB(PRINT_DEBUG,"E");
    
    //Message analysis
    if(recv_string.length()>0) handleMessage(recv_string, externalServerCommunicationControl.currentTaskQueue);
}


MAKE_MODULE(ExternalServerCommunicationController, modeling)
