import sys,os,inspect
from pathlib import Path

def setup_experiment():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)
    sys.path.insert(0, parentparentdir) 
    #print(sys.path)

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.fluents import FluentRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import distance



    domain_path = os.path.join(currentdir, "PDDL", "fond-domain.pddl")
    problem_path = os.path.join(currentdir, "PDDL", "fond-p-0.pddl")
    working_dir = os.path.join(currentdir, "output")


    ActionRegistry(robot_idle_skill="Idle")

    #Ground actions to robot skills
    ActionRegistry()["move-car"] = ("MoveCar")
    ActionRegistry()["changetire"] = ("ChangeTire")

    #Ground :objects in ValueRegistry
    ValueRegistry()["l11"] = "l11"
    ValueRegistry()["l12"] = "l12"
    ValueRegistry()["l13"] = "l13"
    ValueRegistry()["l21"] = "l21"
    ValueRegistry()["l22"] = "l22"
    ValueRegistry()["l31"] = "l31"

    #Ground fluents to a value
    FluentRegistry()["not-flattire"] = False

    print("Creating Policy from domain file: %s with problem file %s" % (domain_path, problem_path))
    try:
        simple_approacher_policy = Policy.build_from_FOND_PDDL(domain_path, problem_path, working_dir)
        simple_approacher_policy.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "policy_preview", Path(os.path.abspath(__file__)).stem+".png"), show_plot = False)
        #policy_handler = PolicyHandler(simple_approacher_dfa, dfa_postprocessing_functions = [remove_initial_dummy_state])
    except AssertionError as e:
        raise e
    else:
        print("OK")

    #nx.draw_networkx(plan.graph, pos)
    #edge_labels = dict([((n1, n2), action)
    #                    for n1, n2, action in plan.graph.edges(data="action")])
    #nx.draw_networkx_edge_labels(plan.graph, pos, edge_labels=edge_labels)
    #plt.show()