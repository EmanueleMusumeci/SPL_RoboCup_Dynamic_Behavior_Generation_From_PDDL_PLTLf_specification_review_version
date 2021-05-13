#include "DeviceCommunicationController.h"
#include "Tools/Modeling/Obstacle.h"
#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#define CONTROL_DEVICE_COMMUNICATION_READ_PORT_BASE 65000
#define CONTROL_DEVICE_COMMUNICATION_WRITE_PORT_BASE 65100
#define PERFORM_KEEPALIVE_CHECK false

#define BUFFER_SIZE 1024

//TODO Client -> Server keepalive

DeviceCommunicationController::DeviceCommunicationController(){

    int write_port_number = CONTROL_DEVICE_COMMUNICATION_WRITE_PORT_BASE + theRobotInfo.number-1;
    this->udp_write_socket.setTarget("127.0.0.1", write_port_number);
    this->udp_write_socket.setBlocking(false);

    int read_port_number = CONTROL_DEVICE_COMMUNICATION_READ_PORT_BASE + theRobotInfo.number-1;
    this->udp_read_socket.bind("127.0.0.1", read_port_number);
    this->udp_read_socket.setBlocking(false);
    
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
    ret_string.append("robot_number|");
    ret_string.append(std::to_string(theRobotInfo.number));
    ret_string.append("|robot_pose|");
    ret_string.append(std::to_string(theRobotPose.rotation));
    ret_string.append("|");
    ret_string.append(std::to_string(theRobotPose.translation.x()));
    ret_string.append("|");
    ret_string.append(std::to_string(theRobotPose.translation.y()));
    return ret_string;
    
}

std::string DeviceCommunicationController::ball_position_to_sendable_string(){

    std::string ret_string;
    ret_string.append("robot_number|");
    ret_string.append(std::to_string(theRobotInfo.number));
    ret_string.append("|ball_position|");
    ret_string.append(std::to_string(theBallModel.estimate.position.x()));
    ret_string.append("|");
    ret_string.append(std::to_string(theBallModel.estimate.position.y()));
    return ret_string;
    
}

std::string DeviceCommunicationController::obstacles_to_sendable_string(){

    std::string ret_string;
    std::string obs_string;

    int obs_number = 0;
    
    ret_string.append("robot_number|");
    ret_string.append(std::to_string(theRobotInfo.number));
    ret_string.append("|obstacles|");
    
    for(auto obs : theObstacleModel.obstacles){
        obs_number++;
        if(obs.type == Obstacle::opponent){
            ret_string.append("|obs|");
            ret_string.append(std::to_string(obs.center.x()));
            ret_string.append("|");
            ret_string.append(std::to_string(obs.center.y()));
        }
    }

    return ret_string;
    
}

void DeviceCommunicationController::send_data_string(std::string str)
{
    const char *s_str = str.c_str();
    this->udp_write_socket.write(s_str, str.length());    
}

/* BUGGY, reading the socket from a different function than update gives error free(): double free detected in tcache 2
//Couldn't solve, so I opted for a bad programming style but at least functioning
bool DeviceCommunicationController::read_data_string_from_socket(UdpComm sock, std::string& recv_str)
{
    bool received = false;

    //char* buffer = (char*) malloc(sizeof(char) * BUFFER_SIZE);
    char buffer[100];

    std::cout<<"HELLO"<<std::endl;
    int bt = this->udp_read_socket.read(buffer, 1);
    std::cout << "bt = " << bt <<std::endl;

    if(bt != -1){
        received = true;

        recv_str += bt;
        std::cout<<"Data received"<<std::endl;
    }
    std::cout<<"HELLO"<<std::endl;
    
    //free(buffer);
    return received;
}
*/

void DeviceCommunicationController::update(DeviceCommunicationControl &deviceCommunicationControl) {

    //std::cout<<"A"<<std::endl;
    std::string recv_string;
    
    bool received = false;
    
    char* buffer = (char*) malloc(sizeof(char) * BUFFER_SIZE);
    //char buffer[100];

    std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"HELLO"<<std::endl;
    int bt = this->udp_read_socket.read(buffer, sizeof(char) * BUFFER_SIZE);
    //std::cout << "bt = " << bt <<std::endl;

    if(bt != -1){
        received = true;

        recv_string += std::string(buffer);
        //std::cout<<"Data received"<<std::endl;
    }
    std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"HELLO"<<std::endl;

    free(buffer);
    

    if(received)
    {
        std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"Received string: "<<recv_string<<std::endl;
    }
    else
    {
        std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"Nothing received"<<std::endl;
    }
    
    if(PERFORM_KEEPALIVE_CHECK)
    {
        std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"this->cycles_since_last_keepalive_check: "<<this->cycles_since_last_keepalive_check<<std::endl;
        if(!this->client_alive || this->cycles_since_last_keepalive_check % KEEPALIVE_CHECK_FREQUENCY == 0)
        {
            if(this->awaiting_keepalive_response)
            {
                std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"Checking keepalive response"<<std::endl;
                if(received)
                {
                    std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"Received keepalive response"<<std::endl;
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
                std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"Checking if client is alive"<<std::endl;
                send_data_string(keepalive_check_string());
                this->awaiting_keepalive_response = true;
            }
        }
        //If waiting for a keepalive response, return until one is received
        if(!this->client_alive) return;

        this->cycles_since_last_keepalive_check++;

        //Ignore keepalive responses
        if(recv_string == "yeah") return;

        std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"After keepalive"<<std::endl;
    }

    
    std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"B"<<std::endl;
    if(this->cycles_since_robot_pose_update % ROBOT_POSE_UPDATE_FREQUENCY == 0){
        send_data_string(robot_pose_to_sendable_string());
        this->cycles_since_robot_pose_update = 0;
    }
    this->cycles_since_robot_pose_update++;

    std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"C"<<std::endl;

    
    if(this->cycles_since_ball_update % BALL_POSITION_UPDATE_FREQUENCY == 0){
        send_data_string(ball_position_to_sendable_string());
        this->cycles_since_ball_update = 0;
    }
    this->cycles_since_ball_update++;

    std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"D"<<std::endl;

    if(this->cycles_since_obstacles_update % OBSTACLES_UPDATE_FREQUENCY == 0){
        send_data_string(obstacles_to_sendable_string());
        this->cycles_since_obstacles_update = 0;
    }
    this->cycles_since_obstacles_update++;
    
    std::cout<<"[Robot #"<<theRobotInfo.number<<"]"<<"E"<<std::endl;

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
