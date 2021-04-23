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
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Communication/GameInfo.h"

void StrikerPFModel::draw() const
{
  #ifdef TARGET_SIM

  const Role& theRole = static_cast<const Role&>(Blackboard::getInstance()["Role"]);
  const LibCheck& theLibCheck = static_cast<const LibCheck&>(Blackboard::getInstance()["LibCheck"]);
  const StrikerPFModel& theStrikerPFModel = static_cast<const StrikerPFModel&>(Blackboard::getInstance()["StrikerPFModel"]);
  const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
  const GameInfo& theGameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);

  if(theStrikerPFModel.graphical_debug && theGameInfo.state == STATE_PLAYING && theRole.role == Role::striker)
  {// drawing of the model in the field view
    DEBUG_DRAWING3D("representation:StrikerPFModel", "field")
    {
      /*float max_potential = 0;
      for(const auto node:potential_field)
      {
        if(node.potential.norm() > max_potential) max_potential = node.potential.norm();
      }*/

      int count = 0;
      for(const auto node:potential_field)
      {
        if(theStrikerPFModel.graphical_draw_radius>0 && 
          (theRobotPose.translation - node.position).norm()>theStrikerPFModel.graphical_draw_radius)
          continue;
        
        if(theStrikerPFModel.show_tiles)
        {
          float remapped_norm = theLibCheck.mapToInterval(node.potential.norm(), theStrikerPFModel.graphical_potential_lower_bound, theStrikerPFModel.graphical_potential_upper_bound, 0, 500);
          Vector3f position3d = Vector3f(node.position.x(), node.position.y(), remapped_norm);
          float color_intensity = theLibCheck.mapToInterval(node.potential.norm(), theStrikerPFModel.graphical_potential_lower_bound, theStrikerPFModel.graphical_potential_upper_bound, 0, 255);

          //SPHERE3D("representation:StrikerPFModel", position3d.x(), position3d.y(), position3d.z(), 10, ColorRGBA(color_intensity, color_intensity, color_intensity));
          QUAD3D2("representation:StrikerPFModel", 
                    Vector3f(position3d.x() - theStrikerPFModel.cell_size/2, position3d.y() - theStrikerPFModel.cell_size/2, 0),
                    Vector3f(position3d.x() + theStrikerPFModel.cell_size/2, position3d.y() - theStrikerPFModel.cell_size/2, 0),
                    Vector3f(position3d.x() + theStrikerPFModel.cell_size/2, position3d.y() + theStrikerPFModel.cell_size/2, 0),
                    Vector3f(position3d.x() - theStrikerPFModel.cell_size/2, position3d.y() + theStrikerPFModel.cell_size/2, 0),
                    5,
                    ColorRGBA(color_intensity, color_intensity, color_intensity, 127)
                    );
        }
        else
        {
          Vector3f potential3d = Vector3f(node.potential.x(), node.potential.y(), 0);
          //Vector3f norm3d = Vector3f(node.potential.norm(), node.potential.norm(), 0);
          
          //Vector3f position3d = Vector3f(node.position.x(), node.position.y(), 5 * node.potential.norm());
          Vector3f position3d = Vector3f(node.position.x(), node.position.y(), 0);
          
          float angle = atan2(node.potential.y(), node.potential.x());
          Vector3f eps3d = Vector3f(cos(angle) * theStrikerPFModel.graphical_arrow_length, sin(angle) * theStrikerPFModel.graphical_arrow_length, 0);

          float color_intensity = theLibCheck.mapToInterval(node.potential.norm(), theStrikerPFModel.graphical_potential_lower_bound, theStrikerPFModel.graphical_potential_upper_bound, 0, 255);
          //if(color_intensity>255) color_intensity=255;
          //SPHERE3D("representation:StrikerPFModel", node.position.x(), node.position.y(), node.potential.norm() * theStrikerPFModel.graphical_norm_factor, 10, ColorRGBA(red,green,0,0));
          
          if(theStrikerPFModel.graphical_arrow_length_as_norm) CYLINDERARROW3D("representation:StrikerPFModel", position3d, position3d + potential3d, 10.f, 10.f, 15.f, ColorRGBA(color_intensity, color_intensity, color_intensity));
          else CYLINDERARROW3D("representation:StrikerPFModel", position3d, position3d + eps3d, 10.f, 10.f, 15.f, ColorRGBA(color_intensity, color_intensity, color_intensity));
        }
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