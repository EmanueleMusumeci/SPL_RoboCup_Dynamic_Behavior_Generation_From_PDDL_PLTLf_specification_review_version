/**
 * @file LibPotentialFields.h
 *
 * This file defines a representation that checks some behavior control properties
 *
 * @author Daniel Krause
 */

#pragma once

#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Modeling/NodePF.h"
#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Representations/Modeling/ObstacleModel.h"

STREAMABLE(LibPotentialFields,
{

  /** Provides the distance between 2 Pose2f **/
  FUNCTION(float(Pose2f p1, Pose2f p2)) distance;

  /** Computes the attractive field for the striker **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, Vector2f goal, float RO, float Kap, float Kbp, float Kr,
                                                    float TEAMMATE_CO, float ETA, float GAMMA)) computeStrikerAttractivePF;

  /** Computes the repulsive field for the striker **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, Vector2f source_pos, float RO, float Kap, float Kbp, float Kr,
                                                    float TEAMMATE_CO, float ETA, float GAMMA)) computeStrikerRepulsivePF;

  /** Initializes an empty PF **/
  FUNCTION(std::vector<NodePF>(float cell_size, float FIELD_BORDER_OFFSET)) initializePFAllField;

 /** Initializes an empty PF spanning the whole field **/
  FUNCTION(std::vector<NodePF> (float cell_size, Vector2f field_center, float field_radius, float FIELD_BORDER_OFFSET)) initializePFAroundPoint;

  /** Computes an artificial potential field based on the provided attractive and repulsive fields **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field)) computePFAllField;

  /** Computes an artificial potential field based on the provided attractive and repulsive fields, in a certain radius given a center point **/
  FUNCTION(std::vector<NodePF>(std::vector<NodePF>& potential_field, std::vector<NodePF> attractive_field, std::vector<NodePF> repulsive_field)) computePFAroundPoint;

  FUNCTION(float(float value, float fromIntervalMin, float fromIntervalMax, float toIntervalMin, float toIntervalMax)) mapToInterval;

  FUNCTION(bool(float currentValue, float target, float bound)) isValueBalanced;
  FUNCTION(float(float x, float y)) angleToTarget;
  FUNCTION(float(float x, float y)) norm;

  FUNCTION(Pose2f(float x, float y)) glob2Rel;
  FUNCTION(float(float x)) radiansToDegree;

  FUNCTION(Pose2f(float x, float y)) rel2Glob,

});
