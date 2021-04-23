/**
 * @file Modules/Modeling/BallCarrierPFModel/BallCarrierPFModelProvider.cpp
 *
 * This module provides the Artificial Potential Field for the striker
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include <iostream>
#include "BallCarrierPFModelProvider.h"
#include "Platform/Time.h"

BallCarrierPFModelProvider::BallCarrierPFModelProvider() :
    last_attractive_field_update(0.f),
    last_repulsive_field_update(0.f),
    last_potential_field_update(0.f)
{}

void BallCarrierPFModelProvider::update(BallCarrierPFModel& ballCarrierPFModel)
{
    //std::cout<<"BallCarrierPFModelProvider"<<std::endl;
    if(theGameInfo.state == STATE_PLAYING && theRole.role == Role::striker)
    {
        ballCarrierPFModel.graphical_debug = (GRAPHICAL_DEBUG==1 ? true : false);
        ballCarrierPFModel.show_tiles = (SHOW_TILES==1 ? true : false);

        ballCarrierPFModel.graphical_norm_factor = GRAPHICAL_NORM_FACTOR;
        ballCarrierPFModel.graphical_potential_upper_bound = GRAPHICAL_POTENTIAL_UPPER_BOUND;
        ballCarrierPFModel.graphical_potential_lower_bound = GRAPHICAL_POTENTIAL_LOWER_BOUND;
        //ballCarrierPFModel.graphical_draw_radius = GRAPHICAL_DRAW_RADIUS;

        ballCarrierPFModel.graphical_min_mesh_height = GRAPHICAL_MAX_MESH_HEIGHT;
        ballCarrierPFModel.graphical_max_mesh_height = GRAPHICAL_MIN_MESH_HEIGHT;

        ballCarrierPFModel.graphical_arrow_length_as_norm = (GRAPHICAL_ARROW_LENGTH_AS_NORM==1 ? true : false);
        ballCarrierPFModel.graphical_arrow_length = GRAPHICAL_ARROW_LENGTH;

        ballCarrierPFModel.field_border_offset = FIELD_BORDER_OFFSET;

        ballCarrierPFModel.max_cell_size = MAXIMUM_CELL_SIZE;
        ballCarrierPFModel.min_cell_size = MINIMUM_CELL_SIZE;
        
        ballCarrierPFModel.min_field_radius = MINIMUM_FIELD_RADIUS;
        ballCarrierPFModel.max_field_radius = MAXIMUM_FIELD_RADIUS;
        
        ballCarrierPFModel.RO = RO;
        ballCarrierPFModel.Kap = Kap;
        ballCarrierPFModel.Kbp = Kbp;
        ballCarrierPFModel.Kr = Kr;
        ballCarrierPFModel.TEAMMATE_CO = TEAMMATE_CO;
        ballCarrierPFModel.ETA = ETA;
        ballCarrierPFModel.GAMMA = GAMMA;
        
        float current_time = Time::getCurrentSystemTime(); //in milliseconds

        if(last_potential_field_update == 0.f || current_time - last_potential_field_update > POTENTIAL_FIELD_DELAY)
        {    
            std::cout<<"Initialize potential field"<<std::endl;

            ballCarrierPFModel.current_field_center = theBallCarrierModel.dynamicTarget.translation;
            std::cout<<"\tballCarrierPFModel.current_field_center: ("<<ballCarrierPFModel.current_field_center.x()<<", "<<ballCarrierPFModel.current_field_center.y()<<")"<<std::endl;
            
            //Potential field radius is inversely proportional to distance from field borders  
            float radius_x = theLibCheck.mapToInterval(std::abs(ballCarrierPFModel.current_field_center.x()), 0.f, theFieldDimensions.xPosOpponentGroundline, 0.f, ballCarrierPFModel.max_field_radius - ballCarrierPFModel.min_field_radius);
            float radius_y = theLibCheck.mapToInterval(std::abs(ballCarrierPFModel.current_field_center.y()), 0.f, theFieldDimensions.yPosLeftSideline, 0.f, ballCarrierPFModel.max_field_radius - ballCarrierPFModel.min_field_radius);
            std::cout<<"radius_x: "<<radius_x<<std::endl;
            std::cout<<"radius_y: "<<radius_y<<std::endl;
            ballCarrierPFModel.current_field_radius = ballCarrierPFModel.max_field_radius - (radius_x>radius_y ? radius_x : radius_y);
            //ballCarrierPFModel.current_field_radius = ballCarrierPFModel.max_field_radius;
            std::cout<<"\tballCarrierPFModel.current_field_radius: "<<ballCarrierPFModel.current_field_radius<<std::endl;

            //Potential field cell size is inversely proportional to distance from field borders
            float cell_size_x = theLibCheck.mapToInterval(std::abs(ballCarrierPFModel.current_field_center.x()), 0.f, theFieldDimensions.xPosOpponentGroundline, 0.f, ballCarrierPFModel.max_cell_size - ballCarrierPFModel.min_cell_size);
            float cell_size_y = theLibCheck.mapToInterval(std::abs(ballCarrierPFModel.current_field_center.y()), 0.f, theFieldDimensions.yPosLeftSideline, 0.f, ballCarrierPFModel.max_cell_size - ballCarrierPFModel.min_cell_size);
            ballCarrierPFModel.current_cell_size = ballCarrierPFModel.max_cell_size - (cell_size_x>cell_size_y ? cell_size_x : cell_size_y);
            std::cout<<"\tballCarrierPFModel.current_cell_size: "<<ballCarrierPFModel.current_cell_size<<std::endl;
            
            ballCarrierPFModel.potential_field = theLibPotentialFields.initializePFAroundPoint(ballCarrierPFModel.current_cell_size, ballCarrierPFModel.current_field_center, ballCarrierPFModel.current_field_radius, ballCarrierPFModel.field_border_offset);

            std::cout<<"Compute attractive field"<<std::endl;
            ballCarrierPFModel.attractive_field = theLibPotentialFields.computeStrikerAttractivePF(ballCarrierPFModel.potential_field, theLibCheck.goalTarget(false), RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
            last_attractive_field_update = Time::getCurrentSystemTime();

            std::cout<<"Compute repulsive field"<<std::endl;
            ballCarrierPFModel.repulsive_field = theLibPotentialFields.computeStrikerRepulsivePF(ballCarrierPFModel.potential_field, ballCarrierPFModel.current_field_center, RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
            last_repulsive_field_update = Time::getCurrentSystemTime();

            std::cout<<"Compute potential field"<<std::endl;
            //Potential field using the goalTarget with shootASAP mode set to false
            ballCarrierPFModel.potential_field = theLibPotentialFields.computePFAroundPoint(ballCarrierPFModel.potential_field, ballCarrierPFModel.attractive_field, ballCarrierPFModel.repulsive_field);
            last_potential_field_update = Time::getCurrentSystemTime();

            std::cout<<ballCarrierPFModel.potential_field.size()<<std::endl;
            for(const auto node: ballCarrierPFModel.potential_field)
            {
                std::cout<<"Node: Position: "<<node.position<<" Potential: "<<node.potential<<std::endl;
            }
            std::cout<<std::endl;
        }
    }

}

MAKE_MODULE(BallCarrierPFModelProvider, modeling)