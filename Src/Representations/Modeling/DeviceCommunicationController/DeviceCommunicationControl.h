/**
 * @file Representations/Modeling/DeviceCommunicationController/DeviceCommunicationControl.h
 *
 * Declaration of struct DeviceCommunicationControl to hold all useful info to receive commands 
 * and manage the communication protocol
 * 
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct DeviceCommunicationControl
 *
 * This representation is used to store received commands and manage the communication protocol
 * 
 * Struct containing modeling info about the opponent goal targetable areas
 */

STREAMABLE(DeviceCommunicationControl,
{

  /** Draws model on the field */
  void draw() const,

  //Loaded from deviceCommunicationControl.cfg, see deviceCommunicationControl for explanation

  (bool) graphicalDebug,
});
