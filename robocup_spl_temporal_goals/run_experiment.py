import argparse
import sys, os

from twisted.internet import reactor

from lib.utils import find_similar_strings
from communication.setup_env import setup
import experiments
from lib.dfa.dfa_handler import DFAHandler
from lib.plan.policy_handler import PolicyHandler

from lib.constraints import ask_additional_constraints, match_string_with_constraint_template
from lib.experiment import ExperimentType

#from GUI.shell import InputShell


if __name__ == "__main__":
    #https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
    dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "experiments")
    sys.path.append(dir)

    from experiments import experiments
    loaded_experiments = {str(experiment_module.__name__): experiment_module for experiment_module in experiments}
    

    parser = argparse.ArgumentParser(description='Run an experiment (which has to be contained in a subdirectory inside the "experiments" directory).')
    parser.add_argument('experiment', type=str, help='Use to specify the experiment name in the format "<experiment_subfolder>.<experiment_name>". The name has to refer to a "<experiment_name>.py" file of the same name inside the subdirectory "<experiment_subfolder>" of the "experiments" folder.')
    parser.add_argument('--localhost', '-l', action="store_true", help='Use to tell if the robot is simulated.')
    parser.add_argument('--simulator', '-s', action="store_true", help='Use to tell if the robot is simulated.')
    #parser.add_argument('--frontend', '-f', action="store_true", help='Use to tell if the graphical frontend is to be used.')
    #parser.add_argument('--GUI', '-g', action="store_true", help='Use to tell if the graphical frontend is to be used.')
    #parser.add_argument('--shell', '-s', action="store_true", help='Use to launch a separate shell for interactive behavior conditioning through PLTLf constraints.')
    
    parser.add_argument('--opponent_team', '-o', action="store_true", help='This server instance controls the opponent team of the GameFast1vs1.ros scenario.')
    parser.add_argument('--ask_constraints', '-c', action="store_true", help='Use to require constraints in advance for each role for interactive behavior conditioning through PLTLf constraints.')
    parser.add_argument('--specify_constraints_by_role', '-C', nargs="+", help='Use to specify constraints in advance for each role for behavior conditioning through PLTLf constraints.')

    args = parser.parse_args()
    


    ''' 
    ____________________
    |                   |
    |  LOAD EXPERIMENT  |
    |___________________|

    '''

    experiment_name = "experiments."+args.experiment
    
    if experiment_name not in loaded_experiments.keys():
        similar_strings = find_similar_strings(experiment_name, loaded_experiments.keys())
        if similar_strings:
            print("Unknown experiment '%s'. Maybe you meant '%s'?" % (experiment_name, similar_strings[0]))
        else:
            print("Unknown experiment '%s'." % (experiment_name))
        exit()
    
    chosen_experiment = loaded_experiments[experiment_name]

    #Check that the experiment module has all necessary functions
    assert hasattr(chosen_experiment, "setup")
    assert hasattr(chosen_experiment, "get_problem_name")
    assert hasattr(chosen_experiment, "get_robot_formation")
    assert hasattr(chosen_experiment, "role_to_generation_data")
    assert hasattr(chosen_experiment, "initialize_registries")

    #Check that the module has a get_robot_formation method
    robot_formation = chosen_experiment.get_robot_formation()

    #Initialize registries with correct variables
    chosen_experiment.initialize_registries()


        #for role_constraint_tuple in constraints_by_roles_tuples:
        #    match_string_with_constraint_template()

    #Ask for additional constraints to the goal of each robot
    if chosen_experiment.get_experiment_type() == ExperimentType.CONSTRAINABLE_POLICY:

        policy_generation_data = chosen_experiment.role_to_generation_data()

        additional_constraints = {}
        if args.specify_constraints_by_role:
            assert len(args.specify_constraints_by_role) % 2 == 0, "When specifying constraints by role, the constraints should be specified in tuples \"role, '<constraint>'\""
            constraints_by_roles_tuples = zip(args.specify_constraints_by_role[::2], args.specify_constraints_by_role[1::2])
            for role_constraint_tuple in constraints_by_roles_tuples:
                assert role_constraint_tuple[0] in policy_generation_data.keys(), "No role "+role_constraint_tuple[0]+" featured in this experiment"
                if role_constraint_tuple[0] not in additional_constraints:
                    additional_constraints[role_constraint_tuple[0]] = [match_string_with_constraint_template(role_constraint_tuple[1], policy_generation_data[role_constraint_tuple[0]]["constrainable_predicates"])]
                else:
                    additional_constraints[role_constraint_tuple[0]].append(match_string_with_constraint_template(role_constraint_tuple[1], policy_generation_data[role_constraint_tuple[0]]["constrainable_predicates"]))
                        
        if args.ask_constraints:
            for role, constraints in ask_additional_constraints(role_to_generation_data = policy_generation_data).items():
                additional_constraints[role].extend(constraints)
        
        plan_handlers = chosen_experiment.setup(additional_constraints)

    else:
        plan_handlers = chosen_experiment.setup()
    
    #Check that the handlers are of the correct type(s)
    assert isinstance(plan_handlers, dict)
    for role, handler in plan_handlers.items():
        assert isinstance(role, str)
        assert isinstance(handler, PolicyHandler) or isinstance(handler, DFAHandler)

    localhost = args.localhost or args.simulator

    
    ''' 
    __________________________
    |                         |
    |   SETUP COMMUNICATION   |
    |_________________________|

    '''

    #frontend = args.frontend or args.GUI
    #behavior_controller = setup(robot_formation, localhost, frontend)
    
    base_framework_port = 65000
    if args.opponent_team:
        base_framework_port = 64000

    #Setup behavior controller and pass policy handler
    behavior_controller = setup(robot_formation, BASE_FRAMEWORK_UDP_PORT = base_framework_port, USE_LOCALHOST = localhost)

    #if args.shell or args.constraints:
    #    shell = InputShell()
    #else:
    for role, handler in plan_handlers.items():
        #We need to first updateRobotRole as, in normal conditions, we would already know the robot role as it is announced as soon as the robot connects
        behavior_controller.update_plan(handler, robot_role=role)

    behavior_controller.start()

            