import os

from lib.dfa.dfa import DFA
from lib.dfa.dfa_handler import DFAHandler, remove_initial_dummy_state
from lib.registries.action import ActionRegistry
from lib.registries.literals import LiteralRegistry
from lib.registries.values import ValueRegistry
from lib.utils import distance
from pathlib import Path

def setup_experiment():
    ''' 
    ___________________
    |                  |
    |  DFA Experiment  |
    |__________________|

    '''

    #EXPERIMENT: Try to realize a patrolling behavior through 4 waypoints. Actions use "static parameters" (values of parameters are not computed functionally through FunctionalValues).
    #This experiment is supposed to show why it is necessary to give actions the possibility to use "dynamic parameters" that are computed through functions, which leads to
    # 1) More manageable DFAs
    # 2) Mutual exclusion of actions is required!!!
    # 3) Also shows the necessity for a DFA post-processing step in which we delete all edges having no actions or only negated actions


    #Setup ValueRegistry
    ValueRegistry()["waypoint_distance_threshold"] = 600
    ValueRegistry()["waypoint1"] = (-1500, -1500)
    ValueRegistry()["waypoint2"] = (-1500, 1500)


    #Setup LiteralRegistry
    def striker_has_reached_waypoint_1(striker_position, waypoint_distance_threshold):
        return distance(striker_position, ValueRegistry()["waypoint1"]) < waypoint_distance_threshold
    LiteralRegistry().add_function(striker_has_reached_waypoint_1)

    def striker_has_reached_waypoint_2(striker_position, waypoint_distance_threshold):
        return distance(striker_position, ValueRegistry()["waypoint2"]) < waypoint_distance_threshold
    LiteralRegistry().add_function(striker_has_reached_waypoint_2)

    '''
    def is_striker_next_waypoint_1(striker_next_waypoint):
        return striker_next_waypoint == "waypoint1"
    LiteralRegistry().add_function(is_striker_next_waypoint_1)

    def is_striker_next_waypoint_2(striker_next_waypoint):
        return striker_next_waypoint == "waypoint2"
    LiteralRegistry().add_function(is_striker_next_waypoint_2)
    '''

    #Setup ActionRegistry
    ActionRegistry(robot_idle_skill="Idle")
    ActionRegistry()["action_go_to_waypoint_1"] = ("ReachPosition", [("positionX", ValueRegistry()["waypoint1"][0]), ("positionY", ValueRegistry()["waypoint1"][1])])
    ActionRegistry()["action_go_to_waypoint_2"] = ("ReachPosition", [("positionX", ValueRegistry()["waypoint2"][0]), ("positionY", ValueRegistry()["waypoint2"][1])])
    
    #Patrol with static params and mutual exclusion DFA
    ltl_formula_str = "G(\
        (striker_has_reached_waypoint_1 && !striker_has_reached_waypoint_2 && action_go_to_waypoint_2) ||\
            (striker_has_reached_waypoint_2 && !striker_has_reached_waypoint_1 && action_go_to_waypoint_1) ||\
                (!striker_has_reached_waypoint_1 && !striker_has_reached_waypoint_2 && action_go_to_waypoint_1) \
                && (striker_has_reached_waypoint_1 -> !striker_has_reached_waypoint_2)\
                && (striker_has_reached_waypoint_2 -> !striker_has_reached_waypoint_1)\
        )"
    print("Creating DFA from formula: %s with post-processing step 'remove_initial_dummy_state'" % (ltl_formula_str))
    try:
        dfa = DFA.DFA_from_LTL_formula_string(ltl_formula_str)
        dfa.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfa_preview", Path(os.path.abspath(__file__)).stem+".png"), show_plot = False)
        dfa_handler = DFAHandler(dfa, dfa_postprocessing_functions = [remove_initial_dummy_state])
    except AssertionError as e:
        raise e
    else:
        print("OK")
    
    return {"striker" : dfa_handler}