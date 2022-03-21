import argparse
import sys, os

from twisted.internet import reactor

from lib.utils import find_similar_strings
from communication.setup_env import setup
import experiments


if __name__ == "__main__":
    #https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
    dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(dir)

    from experiments import experiments
    loaded_experiments = {str(experiment_module.__name__): experiment_module for experiment_module in experiments}
    
    behavior_controller = setup()

    experiment_name = "patrolling_with_static_waypoints_with_mutual_exclusion_stateful"

    experiment_name = "experiments."+experiment_name
    if experiment_name not in loaded_experiments.keys():
        similar_strings = find_similar_strings(experiment_name, loaded_experiments.keys())
        if similar_strings:
            print("Unknown experiment '%s'. Maybe you meant '%s'?" % (experiment_name, similar_strings[0].split(".")[-1]))
        else:
            print("Unknown experiment '%s'.")
        exit()
            
    dfa_handler = loaded_experiments[experiment_name].setup_experiment()

    #We need to first updateRobotRole as, in normal conditions, we would already know the robot role as it is announced as soon as the robot connects
    behavior_controller.updateDFA(dfa_handler, robot_role= "striker")

    behavior_controller.start()

            