import argparse
import sys, os

from twisted.internet import reactor

from lib.utils import find_similar_strings
from communication.setup_env import setup
import experiments
from lib.dfa.dfa_handler import DFAHandler
from lib.plan.policy_handler import PolicyHandler


if __name__ == "__main__":
    #https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
    dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "experiments")
    sys.path.append(dir)

    from experiments import experiments
    loaded_experiments = {str(experiment_module.__name__): experiment_module for experiment_module in experiments}
    

    parser = argparse.ArgumentParser(description='Run an experiment (which has to be contained in a subdirectory inside the "experiments" directory).')
    parser.add_argument('experiment', type=str, help='Use to specify the experiment name in the format "<experiment_subfolder>.<experiment_name>". The name has to refer to a "<experiment_name>.py" file of the same name inside the subdirectory "<experiment_subfolder>" of the "experiments" folder.')

    args = parser.parse_args()
    
    #experiment_name = "reach_waypoint_then_reach_ball_and_kick_until_goal"
    #experiment_name = "classical_planning_examples.classical_planning_example"

    experiment_name = "experiments."+args.experiment
    
    if experiment_name not in loaded_experiments.keys():
        similar_strings = find_similar_strings(experiment_name, loaded_experiments.keys())
        if similar_strings:
            print("Unknown experiment '%s'. Maybe you meant '%s'?" % (experiment_name, similar_strings[0]))
        else:
            print("Unknown experiment '%s'." % (experiment_name))
        exit()
    
    #Check that the module has a function setup_experiment
    assert hasattr(loaded_experiments[experiment_name], "setup_experiment")
    plan_handlers = loaded_experiments[experiment_name].setup_experiment()

    #Check that the handlers are of the correct type(s)
    assert isinstance(plan_handlers, dict)
    for role, handler in plan_handlers.items():
        assert isinstance(role, str)
        assert isinstance(handler, PolicyHandler) or isinstance(handler, DFAHandler)

    #Setup behavior controller and pass policy handler
    behavior_controller = setup()

    for role, handler in plan_handlers.items():
        #We need to first updateRobotRole as, in normal conditions, we would already know the robot role as it is announced as soon as the robot connects
        behavior_controller.update_plan(handler, robot_role=role)

    behavior_controller.start()

            