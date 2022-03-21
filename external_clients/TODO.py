#TODO: [LATER] CommunicationManager should "discover" available robots on the network somehow 
#   1) Use periodic announcement messages from the robots (sending robot name, ip and role)
#   2) Add NAOCommunicationController dynamically when receiving announcement messages from the robot
#   3) If robot crashes, it will be handled correctly by the NAOCommunicationController
#           Maybe delete the interface after a lot of time has passed since the last message
#   4) If the Python server crashes, no problem... Maybe the robot should go into idle mode
#TODO: [LATER] Users unable to control should have a "passive-only" GUI (showing only the field)
#TODO: Framework will stream booleans while other conditions will be available as templates (only structure, no literals or actions) but initialized by GUI messages carrying literal and action names and parameters 
#      NOTICE: this is all hard-coded for now

#NAOCommunicationController:
#TODO: decide what to do in update_assigned_tasks in case the robot role is unknown

#Framework:
#TODO: Whenever an action is completed, answer to a DFAAction...taskID message with tha DFAActionCompleted(taskID) message
#TODO: Add a keepalive on the robot side to have it reset the task list, when in task mode, if no DFAAction message is received for a while


#Actions:
#TODO: Once the robot completes the current DFA action, it will answer to the DFAAction message with a DFAActionCompleted(taskID) message:
#        use the taskID associated to that action to determine if the action completed message refers to that action
#TODO  Single-Shot actions (actions that are completed only once and cannot be performed again)
#TODO  Use Values in the ValueRegistry as action parameters 
    #TODO Once this is done, update the patrolling experiment to use only ONE action using a dynamic parameter that is the "next_waypoint"
#TODO: Should find a pattern to impose a sequence of actions
#TODO: Add possibility of registry side-effects to actions

#Constant Values/Literals/Action parameters declaration
#TODO: Determine a way to declare values, literals and action parameters as constants in the value/literal/action itself

#ValueRegistry:
#DONE: Might also stream values and make the ValueRegistry accept also "functional" values that compute values from streamed values (like positions relative to robots)
#TODO: Support for iterable (list) values

#LiteralRegistry
#TODO: Might need to declare all literals that are mutually excluded by each literal (see experiment "run_with_dfa_experiment_patrolling_with_static_params")
#       Could actually use "mutual exclusion groups" as an argument to the LTLRule constructor: a list of tuples of literals that are mutually excluding (all literals in a tuple are mutually excluding), so that a "mutual exclusion block" may be automatically generated
#       Should automatically mutually exclude actions (automatic mutual exclusion group for all actions)


#DFAs:
#TODO: In the DFANode __str__ method find a way to order edges so that edges between the same two nodes appear together
#TODO: Post-processing of DFAs:
#1) Prune all edges without an action or with more than one action or with no NON-negated action
#2) Prune all loop edges with True
#3) Might decouple states with loop edges 
# 
# i.e. 
# 
# state 1 with loop transitions
#
# (1) -A-> (1) and (1) -B-> (1)
#
# might become
#
# (1) -A-> (2) and (2) -B-> (1) 

#TODO: Add preliminary checks by inspection to all FunctionalRegistryValues:
#     1) Scan the function body to find names that are present in registry but are not specified as function arguments (this might be a user distraction so we want to prevent it)

#NOTICE:
#TODO: MONA returns a non-minimized version of the DFA 
# while using the option mona_dfa_out returns a minimized version but less parsable (it's returned as a PL clause)
# We could use that one (improves condition checking time) if we find a way to encode the '|' or '&' in the PL formula

#Bugs/Undesired effects:
# 1) When the action completion conditions are already satisfied, the DFA will keep spamming that action over and over, increasing the taskID indefinitely
