/**
 * @file Representations/Modeling/OpponentGoalModel.cpp
 * 
 * Implementation of the graphical debug of the Artificial Potential Fields model
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include <iostream>
#include "StrikerPFModel.h"
#include "Platform/Time.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Modeling/RobotPose.h"

void StrikerPFModel::draw() const
{
  #ifdef TARGET_SIM

  const Role& theRole = static_cast<const Role&>(Blackboard::getInstance()["Role"]);
  const StrikerPFModel& theStrikerPFModel = static_cast<const StrikerPFModel&>(Blackboard::getInstance()["StrikerPFModel"]);
  const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);

  if(theStrikerPFModel.graphical_debug && theRole.role == Role::striker)
  {// drawing of the model in the field view
    DEBUG_DRAWING3D("representation:StrikerPFModel", "field")
    {
      /*float max_potential = 0;
      for(const auto node:potential_field)
      {
        if(node.potential.norm() > max_potential) max_potential = node.potential.norm();
      }*/

      for(const auto node:potential_field)
      {
        if(theStrikerPFModel.graphical_draw_radius>0 && 
          (theRobotPose.translation - node.position).norm()>theStrikerPFModel.graphical_draw_radius)
          continue;
        
        Vector3f potential3d = Vector3f(node.potential.x(), node.potential.y(), 0);
        //Vector3f norm3d = Vector3f(node.potential.norm(), node.potential.norm(), 0);
        
        //Vector3f position3d = Vector3f(node.position.x(), node.position.y(), 5 * node.potential.norm());
        Vector3f position3d = Vector3f(node.position.x(), node.position.y(), 0);
        
        float angle = atan2(node.potential.y(), node.potential.x());
        Vector3f eps3d = Vector3f(cos(angle) * theStrikerPFModel.graphical_arrow_length, sin(angle) * theStrikerPFModel.graphical_arrow_length, 0);

        float color_intensity = 255/(theStrikerPFModel.graphical_potential_upper_bound - theStrikerPFModel.graphical_potential_lower_bound) * (node.potential.norm() - theStrikerPFModel.graphical_potential_lower_bound);
        //if(color_intensity>255) color_intensity=255;
        //SPHERE3D("representation:StrikerPFModel", node.position.x(), node.position.y(), node.potential.norm() * theStrikerPFModel.graphical_norm_factor, 10, ColorRGBA(red,green,0,0));
        if(theStrikerPFModel.graphical_arrow_length_as_norm) CYLINDERARROW3D("representation:StrikerPFModel", position3d, position3d + potential3d, 4.f, 10.f, 10.f, ColorRGBA(color_intensity, color_intensity, color_intensity, 125));
        else CYLINDERARROW3D("representation:StrikerPFModel", position3d, position3d + eps3d, 4.f, 10.f, 10.f, ColorRGBA(color_intensity, color_intensity, color_intensity, 125));
      }

    }
  }
  else 
  {
    //If no graphical debug is requested just use an empty graphical representation to avoid the Syntax Error in SimRobot
    DEBUG_DRAWING3D("representation:StrikerPFModel", "field")
    {
    }
  }
  
  #endif
}