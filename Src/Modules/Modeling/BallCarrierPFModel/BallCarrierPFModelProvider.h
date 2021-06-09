/**
 * @file Modules/Modeling/BallCarrierPFModel/BallCarrierPFModelProvider.h
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
#include "Representations/Modeling/BallCarrierPFModel.h"
#include "Representations/BehaviorControl/BallCarrierModel/BallCarrierModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibPotentialFields.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"

MODULE(BallCarrierPFModelProvider,
{,
    REQUIRES(LibCheck),
    REQUIRES(LibPotentialFields),
    REQUIRES(Role),
    REQUIRES(GameInfo),
    REQUIRES(FieldDimensions),
    USES(BallCarrierModel),
    PROVIDES(BallCarrierPFModel),
    LOADS_PARAMETERS(
    {,
      //goalTarget constants
      (bool) GRAPHICAL_DEBUG,                                      /** Show graphical debug in SimRobot */
      (bool) SHOW_TILES,                                           /** Represent the PF as a tiled floor (otherwise its represented as arrows) */
      (float) GRAPHICAL_NORM_FACTOR,                               /** Multiplicative factor for height of graphical render of potential field node */
      (float) GRAPHICAL_POTENTIAL_UPPER_BOUND,                     /** Graphical upper bound for the color of the potential node */
      (float) GRAPHICAL_POTENTIAL_LOWER_BOUND,                     /** Graphical lower bound for the color of the potential node */

      (float) GRAPHICAL_MIN_MESH_HEIGHT,                           /** Min height of the mesh nodes (if SHOW_TILES is True) */  
      (float) GRAPHICAL_MAX_MESH_HEIGHT,                           /** Max height of the mesh nodes (if SHOW_TILES is True) */

      (bool) GRAPHICAL_ARROW_LENGTH_AS_NORM,                       /** Use norm of the potential node as the length of the arrow */
      (float) GRAPHICAL_ARROW_LENGTH,                              /** Length of the graphical representation of a potential node */

      (float) ATTRACTIVE_FIELD_DELAY,                              /** Delay in milliseconds between two subsequent field computations **/
      (float) REPULSIVE_FIELD_DELAY,                               /** Delay in milliseconds between two subsequent field computations **/
      (float) POTENTIAL_FIELD_DELAY,                               /** Delay in milliseconds between two subsequent field computations **/

      (float) FIELD_BORDER_OFFSET,                                 /** Offset from field border for any potential field cell */

      (float) MAXIMUM_CELL_SIZE,                                   /** Max. potential field cell size */
      (float) MINIMUM_CELL_SIZE,                                   /** Min. potential field cell size */
      (float) MINIMUM_FIELD_RADIUS,                                /** Max. distance radius of field cells from field center */
      (float) MAXIMUM_FIELD_RADIUS,                                /** Max. distance radius of field cells from field center */

      (float) RO,
      (float) Kap,
      (float) Kbp,
      (float) Kr,
      (float) TEAMMATE_CO,
      (float) ETA,
      (float) GAMMA,
      (float) POW,
    }),
});

/**
 * @class BallCarrierPFModelProvider
 * A module that provides the potential field for the striker
 */
class BallCarrierPFModelProvider: public BallCarrierPFModelProviderBase
{
public:
  /** Constructor*/
  BallCarrierPFModelProvider();

private:
  void update(BallCarrierPFModel& ballCarrierPFModel) override;
  float last_attractive_field_update; 
  float last_repulsive_field_update; 
  float last_potential_field_update; 
};
