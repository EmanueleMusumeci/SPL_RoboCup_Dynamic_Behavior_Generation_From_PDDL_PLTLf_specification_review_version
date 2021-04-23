/**
 * @file Representations/Modeling/BallCarrierPFModel.h
 *
 * Declaration of struct BallCarrierPFModel, that provides a model of the artificial
 * potential fields from the point of view of the striker, where both opponents, teammates and field
 * borders contribute to the repulsion field while the ball contributes to the attraction field
 *
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Representations/Modeling/NodePF.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct BallCarrierPFModel
 *
 * This representation is used only for debugging purposes
 * 
 * Struct containing modeling info about computed artificial potential field used in the BallCarrier dynamic point computation
 */

STREAMABLE(BallCarrierPFModel,
{
  /** Draws model on the field */
  void draw() const,

  (bool) graphical_debug,
  (bool) show_tiles,
  (float) graphical_norm_factor,
  (float) graphical_potential_upper_bound,
  (float) graphical_potential_lower_bound,

  (float) graphical_min_mesh_height,
  (float) graphical_max_mesh_height,

  (float) graphical_arrow_length_as_norm,
  (float) graphical_arrow_length,
  
  (float) attractive_field_delay,
  (float) repulsive_field_delay,
  (float) potential_field_delay,
  
  (float) RO,
  (float) Kap,
  (float) Kbp,
  (float) Kr,
  (float) TEAMMATE_CO,
  (float) ETA,
  (float) GAMMA,
  
  (std::vector<NodePF>) attractive_field,
  (std::vector<NodePF>) repulsive_field,
  (std::vector<NodePF>) potential_field,

  (float) field_border_offset,

  (float) max_cell_size,
  (float) min_cell_size,
  (float) current_cell_size,
  
  (float) min_field_radius,
  (float) max_field_radius,
  (float) current_field_radius,

  (Vector2f)(Vector2f::Zero()) current_field_center,
});
