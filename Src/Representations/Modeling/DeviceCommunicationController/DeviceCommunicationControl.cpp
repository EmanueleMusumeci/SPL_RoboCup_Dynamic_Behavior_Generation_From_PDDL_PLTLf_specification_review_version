/**
 * @file Representations/Modeling/DeviceCommunicationControl.cpp
 * 
 * Implementation of the drawing functions for the preview of commands received from the control device
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "DeviceCommunicationControl.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void DeviceCommunicationControl::draw() const
{
  #ifdef TARGET_SIM
  // drawing of the model in the field view
  const DeviceCommunicationControl& theDeviceCommunicationControl = static_cast<const DeviceCommunicationControl&>(Blackboard::getInstance()["DeviceCommunicationControl"]);
  if(theDeviceCommunicationControl.graphicalDebug)
  {
    DEBUG_DRAWING3D("representation:DeviceCommunicationControl", "field")
    {
      const Role& theRole = static_cast<const Role&>(Blackboard::getInstance()["Role"]);
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      const GameInfo& theGameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]); 
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);    
      const BallSpecification& theBallSpecification = static_cast<const BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);
    }
  }
  #endif
}