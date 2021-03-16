/**
 * @file Modules/Modeling/StrikerPFModel/StrikerPFModelProvider.h
 *
 * This file implements a module that provides a model of an Artificial Potential
 * Field for the striker based on distances. The repulsive field is based on opponents
 * and teammates, the attractive field is based on the ball and the goal.
 * 
 * Ported from a previous implementation by Vincenzo Suriani
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/StrikerPFModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"

MODULE(StrikerPFModelProvider,
{,
    REQUIRES(LibCheck),
    REQUIRES(Role),
    REQUIRES(GameInfo),
    PROVIDES(StrikerPFModel),
    LOADS_PARAMETERS(
    {,
      //goalTarget constants
      (bool) GRAPHICAL_DEBUG,                                      /** Show graphical debug in SimRobot */
      (bool) SHOW_TILES,                                           /** Represent the PF as a tiled floor (otherwise its represented as arrows) */
      (float) GRAPHICAL_NORM_FACTOR,                               /** Multiplicative factor for height of graphical render of potential field node */
      (float) GRAPHICAL_POTENTIAL_UPPER_BOUND,                     /** Graphical upper bound for the color of the potential node */
      (float) GRAPHICAL_POTENTIAL_LOWER_BOUND,                     /** Graphical lower bound for the color of the potential node */
      (float) GRAPHICAL_DRAW_RADIUS,                               /** Max distance of drawn potential nodes */ 

      (float) GRAPHICAL_MIN_MESH_HEIGHT,                           /** Min height of the mesh nodes (if SHOW_TILES is True) */  
      (float) GRAPHICAL_MAX_MESH_HEIGHT,                           /** Max height of the mesh nodes (if SHOW_TILES is True) */

      (bool) GRAPHICAL_ARROW_LENGTH_AS_NORM,                       /** Use norm of the potential node as the length of the arrow */
      (float) GRAPHICAL_ARROW_LENGTH,                              /** Length of the graphical representation of a potential node */

      (float) ATTRACTIVE_FIELD_DELAY,                              /** Delay in milliseconds between two subsequent field computations **/
      (float) REPULSIVE_FIELD_DELAY,                               /** Delay in milliseconds between two subsequent field computations **/
      (float) POTENTIAL_FIELD_DELAY,                               /** Delay in milliseconds between two subsequent field computations **/

      (float) CELL_SIZE,                                           /** Potential field cell size */

      (float) RO,
      (float) Kap,
      (float) Kbp,
      (float) Kr,
      (float) TEAMMATE_CO,
      (float) ETA,
      (float) GAMMA,
    }),
});

/**
 * @class StrikerPFModelProvider
 * A module that provides the potential field for the striker
 */
class StrikerPFModelProvider: public StrikerPFModelProviderBase
{
public:
  /** Constructor*/
  StrikerPFModelProvider();

private:
  void update(StrikerPFModel& strikerPFModel) override;
  float last_attractive_field_update; 
  float last_repulsive_field_update; 
  float last_potential_field_update; 
};
