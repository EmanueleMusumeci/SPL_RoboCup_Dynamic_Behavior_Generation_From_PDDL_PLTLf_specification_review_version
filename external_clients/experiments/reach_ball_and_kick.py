from lib.dfa.dfa import DFA, DFAHandler, remove_initial_dummy_state
from lib.registries.action import ActionRegistry
from lib.registries.literals import LiteralRegistry
from lib.registries.values import ValueRegistry
from lib.utils import distance

def setup_experiment():
    ''' 
    ___________________
    |                  |
    |  DFA Experiment  |
    |__________________|

    '''

    #Setup ActionRegistry
    ActionRegistry(robot_idle_skill="Idle")
    ActionRegistry()["action_kick_ball"] = ("Kick", [("positionX", 2000), ("positionY", 0)])
    ActionRegistry()["action_reach_ball"] = "ReachBall"

    #Setup ValueRegistry
    ValueRegistry()["ball_distance_threshold"] = 500

    def striker_distance_from_ball(last_ball_position, striker_position):
        return distance(last_ball_position, striker_position)
    ValueRegistry().add_function(striker_distance_from_ball)

    def is_striker_near_ball(striker_distance_from_ball, ball_distance_threshold):
        return striker_distance_from_ball < ball_distance_threshold
    
    #Setup LiteralRegistry
    LiteralRegistry().add_function(is_striker_near_ball)

    #Simple approacher DFA
    ltl_formula_str = "G((is_striker_near_ball && action_kick_ball) || (!is_striker_near_ball && action_reach_ball))"
    print("Creating DFA from formula: %s with post-processing step 'remove_initial_dummy_state'" % (ltl_formula_str))
    try:
        simple_approacher_dfa = DFA.DFA_from_LTL_formula_string(ltl_formula_str)
        dfa_handler = DFAHandler(simple_approacher_dfa, dfa_postprocessing_functions = [remove_initial_dummy_state])
    except AssertionError as e:
        raise e
    else:
        print("OK")
    
    return dfa_handler