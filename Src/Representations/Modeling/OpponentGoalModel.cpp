/**
 * @file Representations/Modeling/OpponentGoalModel.cpp
 * 
 * Implementation of the drawing functions of the OpponentGoalModel
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "OpponentGoalModel.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void OpponentGoalModel::draw() const
{
  #ifdef TARGET_SIM
  // drawing of the model in the field view
  const OpponentGoalModel& theOpponentGoalModel = static_cast<const OpponentGoalModel&>(Blackboard::getInstance()["OpponentGoalModel"]);
  if(theOpponentGoalModel.graphicalDebug)
  {
    DEBUG_DRAWING3D("representation:OpponentGoalModel", "field")
    {
      const Role& theRole = static_cast<const Role&>(Blackboard::getInstance()["Role"]);
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        
      if(freeGoalTargetableAreas.size()!=0 && (theRole.role==Role::RoleType::striker || theRole.role==Role::RoleType::undefined || theOpponentGoalModel.graphicalDebug)) 
      {
        const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
        int red = 55;
        int green = 255;
        int inc = 200/freeGoalTargetableAreas.size();
        for(const auto& targetableArea : freeGoalTargetableAreas)
        {
          if(theRobotPose.translation.x()>0){
            LINE3D("representation:OpponentGoalModel", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, theFieldDimensions.xPosOpponentGroundline, targetableArea.begin, 10, 1, ColorRGBA(red,green,0));
            LINE3D("representation:OpponentGoalModel", theFieldDimensions.xPosOpponentGroundline, targetableArea.begin, 10, theFieldDimensions.xPosOpponentGroundline, targetableArea.end, 10, 2, ColorRGBA(red,green,0));
            LINE3D("representation:OpponentGoalModel", theRobotPose.translation.x(), theRobotPose.translation.y(), 10, theFieldDimensions.xPosOpponentGroundline, targetableArea.end, 10, 1, ColorRGBA(red,green,0));
            red+=inc;
            green-=inc;
          }
        }
      
        const BallSpecification& theBallSpecification = static_cast<const BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);
      
        SPHERE3D("representation:OpponentGoalModel", theOpponentGoalModel.utilityGoalTarget.translation.x(), theOpponentGoalModel.utilityGoalTarget.translation.y(), 12, theBallSpecification.radius, ColorRGBA(255,0,0,0));
        SPHERE3D("representation:OpponentGoalModel", theOpponentGoalModel.shootASAPGoalTarget.translation.x(), theOpponentGoalModel.shootASAPGoalTarget.translation.y(), 12, theBallSpecification.radius, ColorRGBA(255,255,0,0));
        SPHERE3D("representation:OpponentGoalModel", theFieldDimensions.xPosOpponentGroundline, theOpponentGoalModel.myGazeProjection,10,theBallSpecification.radius, ColorRGBA(0,255,255,0));
      }
      if(theRole.role==Role::RoleType::striker || theRole.role==Role::RoleType::undefined || theOpponentGoalModel.graphicalDebug) LINE3D("representation:OpponentGoalModel", theFieldDimensions.xPosOpponentGroundline - theOpponentGoalModel.goalTargetDistanceThreshold, theFieldDimensions.yPosLeftFieldBorder, 10, theFieldDimensions.xPosOpponentGroundline - theOpponentGoalModel.goalTargetDistanceThreshold, theFieldDimensions.yPosRightFieldBorder, 10, 1, ColorRGBA(255,255,255));
    }
  }
  #endif
}