/**
 * @file Representations/BehaviorControl/BallCarrierModel.cpp
 * 
 * Implementation of the drawing functions of the BallCarrierModel
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include "BallCarrierModel.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/Modeling/BallModel.h"
#include <iostream>

void BallCarrierModel::draw() const
{
  const BallCarrierModel& theBallCarrierModel = static_cast<const BallCarrierModel&>(Blackboard::getInstance()["BallCarrierModel"]);
  if(theBallCarrierModel.graphicalDebug)
  {
    const Role& theRole = static_cast<const Role&>(Blackboard::getInstance()["Role"]);
    const GameInfo& theGameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);
    const LibCheck& theLibCheck = static_cast<const LibCheck&>(Blackboard::getInstance()["LibCheck"]);
    #ifdef TARGET_SIM
    DEBUG_DRAWING3D("representation:BallCarrierModel", "field")
    {
      if(theGameInfo.state == STATE_PLAYING && theRole.role==Role::RoleType::striker)
      {
        // drawing of the model in the field view
        
        for(auto edge: theBallCarrierModel.obstacleAvoidancePlan)
        {
          SPHERE3D("representation:BallCarrierModel", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, edge.fromNode.radius,
            ColorRGBA(255,0,0,100));
          LINE3D("representation:BallCarrierModel", 
            edge.fromNode.center.x(), edge.fromNode.center.y(), 10, 
            edge.toNode.center.x(), edge.toNode.center.y(), 10, 
            5, ColorRGBA(255,0,0,255));
        }
        if(!theBallCarrierModel.isFallbackPath)
        {
          BallCarrierModel::Node prevStep = theBallCarrierModel.ballPath[0];
          SPHERE3D("representation:BallCarrierModel", 
            prevStep.center.x(), prevStep.center.y(), 10, 30,
            ColorRGBA(0,255,0,100));

          for(int i=1; i<theBallCarrierModel.ballPath.size(); i++)
          {
            BallCarrierModel::Node step = theBallCarrierModel.ballPath[i];
            SPHERE3D("representation:BallCarrierModel", 
              step.center.x(), step.center.y(), 10, 30,
              ColorRGBA(0,255,0,100));

            LINE3D("representation:BallCarrierModel", 
              prevStep.center.x(), prevStep.center.y(), 10, 
              step.center.x(), step.center.y(), 10, 
              5, ColorRGBA(0,255,0,255));
              prevStep = step;
          }
          //Draw the dynamic approach point (entry point to the approach area, determined by the chosen target)
          /*CYLINDERARROW3D("representation:BallCarrierModel", 
                theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10,
                theBallCarrierModel.dynamicTarget.translation.x()+60, theBallCarrierModel.dynamicTarget.translation.y()+60, 10,
                10, 50, 20, ColorRGBA(0,0,255,255);
                ) 
          SPHERE3D("representation:BallCarrierModel", 
                theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 30,
                ColorRGBA(0,0,255,255));*/
          //Draw the dynamic target (kick target chosen from the path)
        }
        else
        //If there is no path, use the fallback target
        {
          Vector2f globalBallModel = theLibCheck.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation;
          SPHERE3D("representation:BallCarrierModel", 
            globalBallModel.x(), globalBallModel.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          SPHERE3D("representation:BallCarrierModel", 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            30, ColorRGBA(255,0,0,100));

          LINE3D("representation:BallCarrierModel", 
            globalBallModel.x(), globalBallModel.y(), 10, 
            theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
            5, ColorRGBA(200,200,0,255));
        }
        float ARROW_LENGTH = 100;
        CYLINDERARROW3D("representation:BallCarrierModel", 
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10),
              Vector3f(theBallCarrierModel.dynamicTarget.translation.x()+ARROW_LENGTH*cos(theBallCarrierModel.dynamicTarget.rotation), theBallCarrierModel.dynamicTarget.translation.y()+ARROW_LENGTH*sin(theBallCarrierModel.dynamicTarget.rotation), 10),
              10, 50, 20, ColorRGBA(0,0,255,255));
        SPHERE3D("representation:BallCarrierModel", 
              theBallCarrierModel.dynamicTarget.translation.x(), theBallCarrierModel.dynamicTarget.translation.y(), 10, 
              30, ColorRGBA(0,0,255,255));
        
      }
    }
    #endif
  }
}