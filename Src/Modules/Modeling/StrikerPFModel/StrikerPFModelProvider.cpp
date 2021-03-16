/**
 * @file Modules/Modeling/StrikerPFModel/StrikerPFModelProvider.cpp
 *
 * This module provides the Artificial Potential Field for the striker
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#include <iostream>
#include "StrikerPFModelProvider.h"
#include "Platform/Time.h"

StrikerPFModelProvider::StrikerPFModelProvider() :
    last_attractive_field_update(0.f),
    last_repulsive_field_update(0.f),
    last_potential_field_update(0.f)
{}

void StrikerPFModelProvider::update(StrikerPFModel& strikerPFModel)
{
    if(theGameInfo.state == STATE_PLAYING && theRole.role == Role::striker)
    {
        strikerPFModel.graphical_debug = (GRAPHICAL_DEBUG==1 ? true : false);
        strikerPFModel.show_tiles = (SHOW_TILES==1 ? true : false);

        strikerPFModel.graphical_norm_factor = GRAPHICAL_NORM_FACTOR;
        strikerPFModel.graphical_potential_upper_bound = GRAPHICAL_POTENTIAL_UPPER_BOUND;
        strikerPFModel.graphical_potential_lower_bound = GRAPHICAL_POTENTIAL_LOWER_BOUND;
        strikerPFModel.graphical_draw_radius = GRAPHICAL_DRAW_RADIUS;

        strikerPFModel.graphical_min_mesh_height = GRAPHICAL_MAX_MESH_HEIGHT;
        strikerPFModel.graphical_max_mesh_height = GRAPHICAL_MIN_MESH_HEIGHT;

        strikerPFModel.graphical_arrow_length_as_norm = (GRAPHICAL_ARROW_LENGTH_AS_NORM==1 ? true : false);
        strikerPFModel.graphical_arrow_length = GRAPHICAL_ARROW_LENGTH;

        strikerPFModel.cell_size = CELL_SIZE;

        strikerPFModel.RO = RO;
        strikerPFModel.Kap = Kap;
        strikerPFModel.Kbp = Kbp;
        strikerPFModel.Kr = Kr;
        strikerPFModel.TEAMMATE_CO = TEAMMATE_CO;
        strikerPFModel.ETA = ETA;
        strikerPFModel.GAMMA = GAMMA;
        
        float current_time = Time::getCurrentSystemTime(); //in milliseconds

        if(last_potential_field_update == 0.f)
        {
            strikerPFModel.potential_field = theLibCheck.initialize_PF(CELL_SIZE);
        }

        //MIGHT AS WELL USE SOME SMARTER CRITERION
        if(current_time - last_attractive_field_update > ATTRACTIVE_FIELD_DELAY)
        {
            strikerPFModel.attractive_field = theLibCheck.compute_striker_attractive_PF(theLibCheck.goalTarget(false), RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
            last_attractive_field_update = Time::getCurrentSystemTime();
        }

        if(current_time - last_repulsive_field_update > REPULSIVE_FIELD_DELAY)
        {
            strikerPFModel.repulsive_field = theLibCheck.compute_striker_repulsive_PF(RO, Kap, Kbp, Kr, TEAMMATE_CO, ETA, GAMMA);
            last_repulsive_field_update = Time::getCurrentSystemTime();
        }
        
        if(current_time - last_potential_field_update > POTENTIAL_FIELD_DELAY)
        {
            //Potential field using the goalTarget with shootASAP mode set to false
            strikerPFModel.potential_field = theLibCheck.computePF(strikerPFModel.attractive_field, strikerPFModel.repulsive_field, CELL_SIZE);
            last_potential_field_update = Time::getCurrentSystemTime();
            /*std::cout<<strikerPFModel.potential_field.size()<<std::endl;
            for(const auto node: strikerPFModel.potential_field)
            {
                std::cout<<"Node: Position: "<<node.position<<" Potential: "<<node.potential<<std::endl;
            }*/
        }
    }
}

MAKE_MODULE(StrikerPFModelProvider, modeling)