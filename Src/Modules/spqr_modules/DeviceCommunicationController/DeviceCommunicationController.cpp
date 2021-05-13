#include "DeviceCommunicationController.h"
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

//TODO Client -> Server keepalive

DeviceCommunicationController::DeviceCommunicationController(){

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

std::string DeviceCommunicationController::keepalive_check_string(){
    std::string ret_string;
    ret_string.append("uthere?");
    return ret_string;
}

std::string DeviceCommunicationController::robot_pose_to_sendable_string(){

    std::string ret_string;
    ret_string.append("robot_pose,");
    ret_string.append(std::to_string(theRobotPose.rotation));
    ret_string.append(",");
    ret_string.append(std::to_string(theRobotPose.translation.x()));
    ret_string.append(",");
    ret_string.append(std::to_string(theRobotPose.translation.y()));
    return ret_string;
    
}

std::string DeviceCommunicationController::ball_position_to_sendable_string(){

    Vector2f globalBall = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
    std::string ret_string;
    ret_string.append("ball_position,");
    ret_string.append(std::to_string(globalBall.x()));
    ret_string.append(",");
    ret_string.append(std::to_string(globalBall.y()));
    return ret_string;
    
}

std::string DeviceCommunicationController::obstacles_to_sendable_string(){

    std::string ret_string;
    std::string obs_string;

    int obs_number = 0;
    
    ret_string.append("obstacles");
    
    for(auto obs : theObstacleModel.obstacles){
        obs_number++;
        if(obs.type == Obstacle::opponent){
            ret_string.append(";");
            ret_string.append(std::to_string(obs.center.x()));
            ret_string.append(",");
            ret_string.append(std::to_string(obs.center.y()));
        }
    }

    return ret_string;
}

void DeviceCommunicationController::send_data_string(std::string str, bool prefix_timestamp, bool prefix_robot_number)
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

void DeviceCommunicationController::update(DeviceCommunicationControl &deviceCommunicationControl) {

    //std::cout<<"A"<<std::endl;
    std::string recv_string;
    
    bool received = false;
    
    char* buffer = (char*) malloc(sizeof(char) * BUFFER_SIZE);

    DEBUG_NUMB(PRINT_DEBUG,"HELLO");
    int bt = this->udp_read_socket.read(buffer, sizeof(char) * BUFFER_SIZE);
    //std::cout << "bt = " << bt <<std::endl;

    if(bt != -1){
        received = true;

        recv_string += std::string(buffer);
        //std::cout<<"Data received"<<std::endl;
    }
    DEBUG_NUMB(PRINT_DEBUG,"HELLO");

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
        DEBUG_NUMB(PRINT_DEBUG,"this->cycles_since_last_keepalive_check: "<<this->cycles_since_last_keepalive_check);
        if(!this->client_alive || this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
        {
            if(this->awaiting_keepalive_response)
            {
                DEBUG_NUMB(PRINT_DEBUG,"Checking keepalive response");
                if(received)
                {
                    DEBUG_NUMB(PRINT_DEBUG,"Received keepalive response");
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
                DEBUG_NUMB(PRINT_DEBUG,"Checking if client is alive");
                send_data_string(keepalive_check_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
                this->awaiting_keepalive_response = true;
            }
        }
        //If waiting for a keepalive response, return until one is received
        if(!this->client_alive) return;

        this->cycles_since_last_keepalive_check++;

        //Ignore keepalive responses
        if(recv_string == "yeah") return;

        DEBUG_NUMB(PRINT_DEBUG,"After keepalive");
    }

    
    DEBUG_NUMB(PRINT_DEBUG,"B");
    if(this->cycles_since_robot_pose_update % ROBOT_POSE_UPDATE_FREQUENCY == 0){
        send_data_string(robot_pose_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
        this->cycles_since_robot_pose_update = 0;
    }
    this->cycles_since_robot_pose_update++;

    DEBUG_NUMB(PRINT_DEBUG,"C");
    
    if(this->cycles_since_ball_update % BALL_POSITION_UPDATE_FREQUENCY == 0){
        send_data_string(ball_position_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
        this->cycles_since_ball_update = 0;
    }
    this->cycles_since_ball_update++;


    DEBUG_NUMB(PRINT_DEBUG,"D");

    if(this->cycles_since_obstacles_update % OBSTACLES_UPDATE_FREQUENCY == 0){
        send_data_string(obstacles_to_sendable_string(), PREFIX_TIMESTAMP, PREFIX_ROBOT_NUMBER);
        this->cycles_since_obstacles_update = 0;
    }
    this->cycles_since_obstacles_update++;
    
    DEBUG_NUMB(PRINT_DEBUG,"E");

    /*if(this->cycles % 200 == 0){
        std::string state_string = data_to_string();
    
        const char *s_str = state_string.c_str();
        this->udp_write_socket.write(s_str, state_string.length());
        char dataBuf[100];
    
        int bt = this->udp_read_socket.read(dataBuf, 1);
        std::cout << "bt = " << bt <<std::endl;
        
        if(bt != -1){
            int action = dataBuf[0] - '0';
            std::cout << "ho letto: " << dataBuf[0] << " azione = " << action << std::endl;
            rla.rl_action = action;
        }
        this->cycles = 0;
    }*/
}

MAKE_MODULE(DeviceCommunicationController, modeling)
