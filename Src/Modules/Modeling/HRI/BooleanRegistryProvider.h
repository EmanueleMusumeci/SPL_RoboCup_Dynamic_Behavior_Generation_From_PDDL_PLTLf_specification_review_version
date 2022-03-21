/**
 * @file BooleanRegistryProvider.h
 *
 * This module keeps track of the state of execution of the Obstacle Avoidance HRI routine 
 *  
 * @author <A href="mailto:musumeci.1653885@studenti.uniroma1.it">Emanuele Musumeci</A>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/HRI/BooleanRegistry.h"
#include "Representations/HRI/TaskController.h"
#include "Representations/Communication/RobotInfo.h"

#include <iostream>
#include <ostream>

MODULE(BooleanRegistryProvider,
{,
    REQUIRES(RobotInfo),
    REQUIRES(TaskController),
    PROVIDES(BooleanRegistry),
    
    LOADS_PARAMETERS(
    {,
      (bool) PRINT_DEBUG,
      
      (bool) ALWAYS_SEND,

    }),
});

/**
 * @class BooleanRegistryProvider
 * A module that provides the boolean registry function to create the message to be sent to the client to update its known booleans from the robot
 */
class BooleanRegistryProvider: public BooleanRegistryProviderBase
{
public:
  /** Constructor */
  BooleanRegistryProvider();

private:
  void update(BooleanRegistry& controller) override;
};