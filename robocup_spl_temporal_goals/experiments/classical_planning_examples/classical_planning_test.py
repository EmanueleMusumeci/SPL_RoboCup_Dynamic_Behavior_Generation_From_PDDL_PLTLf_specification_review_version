
import sys,os,inspect
from pathlib import Path

import networkx as nx
import matplotlib.pyplot as plt

def setup_experiment():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)
    sys.path.insert(0, parentparentdir) 

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import linear_distance, angular_distance

    #from robocup_spl_temporal_goals.third_party.planning_with_past.planners.downward import DownwardPlanner

    domain_path = os.path.join(currentdir, "PDDL", "domain.pddl")
    problem_path = os.path.join(currentdir, "PDDL", "p-0.pddl")
    working_dir = os.path.join(currentdir, "output")


    ActionRegistry(robot_idle_skill="Idle")

    #Ground actions to robot skills
    ActionRegistry().register_action_template("pick-up", "PickUp", [0])
    ActionRegistry().register_action_template("stack", "Stack", [0, 1])
    ActionRegistry().register_action_template("unstack", "Unstack", [0, 1])

    #Ground :objects in ValueRegistry
    ValueRegistry()["d"] = "d"
    ValueRegistry()["b"] = "b"
    ValueRegistry()["a"] = "a"
    ValueRegistry()["c"] = "c"

    print("Creating Plan from domain file: %s with problem file %s" % (domain_path, problem_path))
    try:
        simple_approacher_plan = Policy.build_from_PDDL(domain_path, problem_path, working_dir)
        simple_approacher_plan.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "plan_preview", Path(os.path.abspath(__file__)).stem+".png"), show_plot = False)
        #plan_handler = PolicyHandler(simple_approacher_dfa, dfa_postprocessing_functions = [remove_initial_dummy_state])
    except AssertionError as e:
        raise e
    else:
        print("OK")

    # print the graph
    #pos = nx.spring_layout(plan.graph)
    #nx.draw_networkx(plan.graph, pos)
    #edge_labels = dict([((n1, n2), action)
    #                    for n1, n2, action in plan.graph.edges(data="action")])
    #nx.draw_networkx_edge_labels(plan.graph, pos, edge_labels=edge_labels)
    #plt.show()