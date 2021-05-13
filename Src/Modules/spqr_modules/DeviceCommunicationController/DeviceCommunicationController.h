#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Communication/UdpComm.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Tools/Math/Pose2f.h"

#include "Representations/Modeling/DeviceCommunicationController/DeviceCommunicationControl.h"

#include "Representations/BehaviorControl/Libraries/LibCheck.h"

#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(DeviceCommunicationController,
{,
 USES(BehaviorStatus),
 REQUIRES(BallModel),
 REQUIRES(RobotInfo),
 REQUIRES(LibCheck),
 REQUIRES(TeamData),
 REQUIRES(RobotPose),
 REQUIRES(ObstacleModel),
 REQUIRES(GameInfo),
 USES(ActivationGraph),
 REQUIRES(OwnTeamInfo),
 REQUIRES(FieldBall),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(FieldDimensions),
 PROVIDES(DeviceCommunicationControl),
 
 LOADS_PARAMETERS(
    {,
      (int) KEEPALIVE_CHECK_FREQUENCY,              /** Number of cycles between two different keepalive requests to client device */

      (int) ROBOT_POSE_UPDATE_FREQUENCY,            /** Number of cycles between two different robot pose updates to client device */
      (int) BALL_POSITION_UPDATE_FREQUENCY,         /** Number of cycles between two different ball info updates to client device */
      (int) OBSTACLES_UPDATE_FREQUENCY,             /** Number of cycles between two different obstacles info updates to client device */
    }),
});

class DeviceCommunicationController : public DeviceCommunicationControllerBase
{
private:
    void isDone();

public:
    UdpComm udp_write_socket;
    UdpComm udp_read_socket;
    
    int cycles_since_robot_pose_update;
    int cycles_since_ball_update;
    int cycles_since_obstacles_update;

    int cycles_since_last_keepalive_check;
    
    bool client_alive;
    bool awaiting_keepalive_response;

    void update(DeviceCommunicationControl &deviceCommunicationControl);

    void check_client_alive();

    std::string keepalive_check_string();

    std::string robot_pose_to_sendable_string();
    std::string ball_position_to_sendable_string();
    std::string obstacles_to_sendable_string();

    void send_data_string(std::string str);
    bool read_data_string_from_socket(UdpComm sock, std::string& recv_str);

    DeviceCommunicationController();
};
