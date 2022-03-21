from cProfile import label
from gettext import translation
from operator import is_
from ssl import VerifyFlags
from typing import List, Type, Tuple
import os
import time

import ltlf2dfa
from ltlf2dfa.parser.ltlf import LTLfParser
import graphviz

from lib.registries.literals import *
from lib.registries.action import *
from lib.dfa.LTL import LTLRule
import lib.utils

class DFAEdge:
    #Use of Type["<type_name>"] is necessary because DFANode has not yet been defined
    def __init__(self, from_node : Type['DFANode'], to_node : Type['DFANode'], guard_literals : List[Tuple[Literal, bool]], guard_actions : List[Tuple[Action, bool]] = None):

        for literal_tuple in guard_literals:
            assert isinstance(literal_tuple, Tuple)
            assert isinstance(literal_tuple[0], Literal)
            assert isinstance(literal_tuple[1], bool)
        self.guard_literals = guard_literals

        if guard_actions is None:
            guard_actions = []
        else:
            for action_tuple in guard_actions:
                assert isinstance(action_tuple, Tuple)
                assert isinstance(action_tuple[0], Action)
                assert isinstance(action_tuple[1], bool)
        self.guard_actions = guard_actions
        
        assert isinstance(from_node, DFANode)
        assert isinstance(to_node, DFANode)



        self.from_node = from_node
        self.to_node = to_node

    def get_actions(self) -> List[Tuple[Action, bool]]:
        return self.guard_actions

    def get_non_negated_actions(self) -> List[Tuple[Action, bool]]:
        non_negated_actions = []
        #print(self)
        for action in self.get_actions():
            #print(action)
            if action[1] == True:
                non_negated_actions.append(action)
        return non_negated_actions
    
    def get_literals(self) -> List[Tuple[Literal, bool]]:
        return self.guard_literals
    
    def is_verified(self) -> bool:
        all_verified = True
        for literal_tuple in self.guard_literals:
            
            try:
                current_literal_value = bool(literal_tuple[0])
            except KeyError as ke:
                print("KeyError raised: %s.\n\nCan't evaluate literal %s: no transition is possible." % (str(ke), literal_tuple[0].name_in_registry))
                return False
            except Exception as e:
                raise e

            expected_literal_value = literal_tuple[1]
            this_literal_verified = (current_literal_value == expected_literal_value)
            all_verified = all_verified and this_literal_verified
            if not all_verified:
                break
        return all_verified
    
    def __str__(self):
        if self.guard_literals:
            literals_string = "\n\tLiterals:"
            for literal in self.guard_literals:
                literals_string += "\n\t\t"+("NOT " if not literal[1] else "") + str(literal[0])
        else:
            literals_string = "\n\tLiterals: []"

        if self.guard_actions:
            actions_string = "\n\tActions:"
            for action in self.guard_actions:
                actions_string += "\n\t\t"+("NOT " if not action[1] else "") +str(action[0])
        else:
            actions_string = "\n\tActions: []"

        return "Edge: %s -> %s%s%s" % (self.from_node.node_id, self.to_node.node_id, literals_string, actions_string) + "\n"

    def get_label_string(self):
        if not self.guard_actions and not self.guard_literals:
            return "True"
        label_string = "["
        for i, literal in enumerate(self.guard_literals):
            label_string += (" & " if i > 0 else "") +("~" if not literal[1] else "") + str(literal[0].name_in_registry)
        label_string += "] "
        for i, action in enumerate(self.guard_actions):
            label_string += (" & " if i > 0 else "") +("~" if not action[1] else "") + str(action[0].name_in_registry)
        return label_string

    def __repr__(self):
        return str(self)
        

class DFANode:
    def __init__(self, node_id, edges : List[DFAEdge] = None, is_accepting_state : bool = False):

        assert isinstance(is_accepting_state, bool)

        self.is_accepting_state = is_accepting_state
        self.__outgoing_edges = []
        self.__incoming_edges = []
        if edges is None:
            self.__edges = []
        else:
            self.__edges = edges
            for edge in edges:
                assert isinstance(edge, DFAEdge)
                assert edge.from_node == self or edge.to_node == self
                if edge.from_node == self:
                    self.__outgoing_edges.append(edge)
                if edge.to_node == self:
                    self.__incoming_edges.append(edge)

        self.node_id = node_id
    
    def get_edges(self):
        return self.__edges

    def get_outgoing_edges(self):
        return self.__outgoing_edges

    def get_true_edges(self) -> List[DFAEdge]:
        valid_edges = []
        for edge in self.__edges:
            if edge.is_verified():
                valid_edges.append(edge)
        return valid_edges
    
    def add_edge(self, edge) -> None:
        #print("NODE_ID: "+str(self.node_id)+", ADDING EDGE: "+str(edge))
        assert edge.from_node == self or edge.to_node == self
        self.__edges.append(edge)
        if edge.from_node == self:
            self.__outgoing_edges.append(edge)
        if edge.to_node == self:
            self.__incoming_edges.append(edge)
    
    def remove_edge(self, edge_to_remove) -> None:
        assert edge_to_remove.from_node.node_id == self.node_id or edge_to_remove.to_node.node_id == self.node_id
        if edge_to_remove.from_node == self:
            self.__outgoing_edges.remove(edge_to_remove)

        if edge_to_remove.to_node == self:
            self.__incoming_edges.remove(edge_to_remove)
    
    def get_edges(self) -> List[DFAEdge]:
        return self.__edges

    def __str__(self):
        edges_string = ""
        #for edge in self.__edges:
        for edge in self.__outgoing_edges:
            edges_string += str(edge)
        edges_string = lib.utils.tab_all_lines_in_string(edges_string, times = 1)
        edges_string = "\nOutgoing edges:\n" + edges_string
        #edges_string = "\nEdges:\n" + edges_string
        return "\nNode ID: %s%s%s" % (str(self.node_id), (" (accepting)" if self.is_accepting_state else ""), edges_string)

    def __repr__(self):
        return str(self)
    
    def __eq__(self, other_node : Type['DFANode']):
        assert isinstance(other_node, DFANode)
        return self.node_id == other_node.node_id

class DFA:
    def __init__(self, states : List[DFANode], accepting_states : List[DFANode], rejecting_states : List[DFANode], initial_state : DFANode = None):
        self.states = states
        self.accepting_states = accepting_states
        self.rejecting_states = rejecting_states
        
        self.initial_state = initial_state
        if initial_state is None:
            self.initial_state = self.states[0]
        

    def add_edge(node1 : DFANode, node2 : DFANode, literals : List[Literal], actions : List[Action] = None) -> None:
        new_edge = DFAEdge(node1, node2, guard_literals=literals, guard_actions=actions)
        node1.add_edge(new_edge)
        node2.add_edge(new_edge)
    
    def remove_edge(edge : DFAEdge):
        edge.from_node.remove_edge(edge)
        #Call twice only if the edge is not a loop
        if edge.from_node != edge.to_node:
            edge.to_node.remove_edge(edge)

    @classmethod
    def build_from_MONA_string(cls, dfa_MONA_string, mutual_exclusion_groups = []):
        #print(dfa_MONA_string)
        free_variables_list = []
        actions = {}
        literals = {}
        initial_state_id = None
        states = {}
        accepting_states = {}
        rejecting_states = {}
        for line in dfa_MONA_string.split("\n"):
            #Parse free variables
            if line.startswith("DFA for formula with free variables: "):
                #NOTICE: split the free variables and filter out empty elements (from the string split)
                free_variables_list = list(filter(lambda list_element : bool(list_element), line.replace("DFA for formula with free variables: ", "").split(" ")))
                for variable in free_variables_list:
                    if variable in LiteralRegistry():
                        literals[variable] = LiteralRegistry().get_instance(variable)
                    elif variable in ActionRegistry():
                        actions[variable] = ActionRegistry().get_instance(variable)
                    else:
                        raise AssertionError("Unknown variable: %s.\nAll free variables in the DFA should be registered in the ActionRegistry (if they're actions) or in the LiteralRegistry (if they're literals)" % (variable))

            #Parse initial state id
            elif line.startswith("Initial state: "):
                initial_state_id = int(list(filter(lambda list_element : bool(list_element), line.replace("Initial state: ", "").split(" ")))[0])
                states[initial_state_id] = DFANode(initial_state_id)

            #Parse accepting states ids
            elif line.startswith("Accepting states: "):
                assert initial_state_id is not None
                accepting_states_list = list(filter(lambda list_element : bool(list_element), line.replace("Accepting states: ", "").split(" ")))
                for state_id in accepting_states_list:
                    state_id = int(state_id)
                    #Special case for the initial state: it was necessarily created already so it must be set as accepting state
                    if state_id == initial_state_id:
                        states[initial_state_id].is_accepting_state = True
                    else:
                        states[state_id] = DFANode(state_id, is_accepting_state=True)
                    accepting_states[state_id] = states[state_id]

            #Parse rejecting states ids
            elif line.startswith("Rejecting states: "):
                assert initial_state_id is not None
                rejecting_states_list = list(filter(lambda list_element : bool(list_element), line.replace("Rejecting states: ", "").split(" ")))
                for state_id in rejecting_states_list:
                    state_id = int(state_id)
                    states[state_id] = DFANode(state_id)
                    rejecting_states[state_id] = states[state_id]

            elif line.startswith("State"):
                assert initial_state_id is not None
                transition_string_tokens = list(filter(lambda list_element : bool(list_element), line.replace("State ", "").split(" ")))
                from_state_id = int(transition_string_tokens[0][:-1])
                #print("FROM_STATE_ID %d" % (from_state_id))
                assert from_state_id in states.keys()
                from_state = states[from_state_id]

                to_state_id = int(transition_string_tokens[-1])
                #print("TO_STATE_ID %d" % (to_state_id))
                assert to_state_id in states.keys()
                to_state = states[to_state_id]
                
                edge_actions = []
                edge_literals = []

                #For each guard variable in the transition condition
                for i, state_variable in enumerate(transition_string_tokens[1]):
                    #print(state_variable)
                    #If the guard variable value is taken into consideration on the current edge
                    if state_variable!="X":
                        should_be_verified = (True if state_variable=="1" else False)
                        variable_name = free_variables_list[i]
                        if variable_name in actions.keys():
                            edge_actions.append((ActionRegistry().get_instance(variable_name), should_be_verified))
                        elif variable_name in literals.keys():
                            edge_literals.append((LiteralRegistry().get_instance(variable_name), should_be_verified))
                        else:
                            raise AssertionError("Unknown variable: %s.\nAll free variables in the DFA should be registered in the ActionRegistry (if they're actions) or in the LiteralRegistry (if they're literals)" % (variable_name))

                #print(edge_actions)
                #print(edge_literals)
                #print(from_state)
                #print(to_state)
                
                #Create edge with the information obtained
                new_edge = DFAEdge(from_node = from_state, to_node = to_state, guard_literals = edge_literals, guard_actions = edge_actions)
                from_state.add_edge(new_edge)
                #Avoid adding edge twice in case of loops
                if to_state_id != from_state_id:
                    to_state.add_edge(new_edge)
                
        
        assert actions
        assert literals
        assert initial_state_id is not None
        assert accepting_states

        #Create DFA with the collected information
        dfa = DFA(states, accepting_states, rejecting_states, initial_state=states[initial_state_id])
        return dfa

    @classmethod
    def DFA_from_LTL_formula_string(cls, formula_string):
        ltl_rule = LTLRule(formula_string)

        dfa_MONA_string = ltl_rule.formula.to_dfa(mona_dfa_out=True)
        return DFA.build_from_MONA_string(dfa_MONA_string)

    @classmethod
    def DFA_from_LTL_rule(cls, ltl_rule_instance : LTLRule):
        assert isinstance(ltl_rule_instance, LTLRule)

        dfa_MONA_string = ltl_rule_instance.formula.to_dfa(mona_dfa_out=True)
        return DFA.build_from_MONA_string(dfa_MONA_string)

#TODO: Implement dfa printing (might also implement a method for a graphical render of the DFA)
    def __str__(self):
        dfa_string = ""
        for state_id, state in self.states.items():
            dfa_string += str(state)
        return dfa_string

    def __repr__(self):
        return str(self)

    def plot(self, save_to : str, show_plot = False):
        dot_format_string = '\
            digraph MONA_DFA {\n \
            rankdir = LR;\n \
            center = true;\n \
            size = "14.5,20.5";\n \
            edge [fontname = Courier];\n \
            node [height = .5, width = .5];\n \
            node [shape = doublecircle]; 1;\n \
            node [shape = circle]; 1;\n \
            init [shape = plaintext, label = ""];\n'

        dot_format_string += "init -> "+str(self.initial_state.node_id)+";\n"
        for state_id, state in self.states.items():
            for edge in state.get_outgoing_edges():
                dot_format_string += str(edge.from_node.node_id) + " -> " + str(edge.to_node.node_id) + ' [label="' + edge.get_label_string() + '"];\n'
        dot_format_string += "}"

        graph_source = graphviz.Source(dot_format_string)
        graph_source.render(outfile=save_to, view=show_plot).replace('\\', '/')

        return dot_format_string

#TODO: get_current_state should 1) Update the DFA 2) return the literals and are actions for the current state (only action is needed but we want this to be more general purpose)
class DFAHandler:
    def __init__(self, dfa, dfa_postprocessing_functions = []):
        if dfa_postprocessing_functions:
            for preprocessing_step in dfa_postprocessing_functions:
                preprocessing_step(dfa)
        self.dfa = dfa
        self.__current_state : DFANode = dfa.initial_state
        self.__current_edge = None

        self.__previous_state = None
        self.__previous_edge = None
        self.__trace = [{"edge" : None, "performed_action" : ActionRegistry().get_instance("action_idle"), "destination_state" : self.__current_state.node_id, "timestamp" : time.time()}]
    
    def get_current_action(self):
        self.update()
        #print(self.__trace)
        return self.__trace[-1]["performed_action"]

    def get_previous_action(self):
        if len(self.__trace) == 1:
            return ActionRegistry()["action_idle"]
        else:
            return self.__trace[-2]["performed_action"]
    
    def get_current_state(self):
        self.update()
        return self.__current_state.node_id

    def get_previous_state(self):
        return self.__previous_state.node_id

    def get_last_transition_timestamp(self):
        return self.__trace[-1]["timestamp"]
    
    def update(self):
        outgoing_edges : List[DFAEdge] = self.__current_state.get_outgoing_edges()

        #Select only edges where all literals are verified
        verified_edges = []
        #print(outgoing_edges)
        for edge in outgoing_edges:
            if edge.is_verified():
                verified_edges.append(edge)

        print(verified_edges)

#TODO: remove this step after the pruning of negated actions has been implemented
        #Filter out edges that have a negated action
        filtered_verified_edges = []
        for edge in verified_edges:
            non_negated_actions = edge.get_non_negated_actions()
            #print(non_negated_actions)
            if not non_negated_actions:
                continue
            #TODO: relax this if multiple actions are supported
            assert len(non_negated_actions) == 1
            #If the action is "negated", ignore this edge
            assert non_negated_actions[0][1] == True
            filtered_verified_edges.append(edge)
        
        assert filtered_verified_edges
        print("\n"+str(filtered_verified_edges))
        
        
        if not filtered_verified_edges:
            print("USING PREVIOUS ACTION")
            chosen_transition = self.__trace[-1]["edge"]
        else:
            #print("CHOOSING BEST ACTION")
            chosen_transition : DFAEdge = self.choose_best_edge(filtered_verified_edges)
        
        #print(chosen_transition)
        if chosen_transition is not None:
    #TODO: relax this if multiple actions are going to be supported        
            assert len(chosen_transition.get_non_negated_actions()) == 1

            chosen_action : Action = chosen_transition.get_non_negated_actions()[0][0]
            destination_state : DFANode = chosen_transition.to_node
        else:
            chosen_action : Action = ActionRegistry()["action_idle"]
            destination_state : DFANode = self.__current_state.node_id

        #If we're not still repeating the same current action
        if chosen_transition != self.__current_edge:
            print("UPDATING TRACE WITH CHOSEN ACTION '%s'" % (str(chosen_action)))
            #Update previous state/edge and trace in case the new one is different from the previous one
            self.__previous_edge = self.__current_edge
            self.__previous_state = self.__current_state

            self.__trace .append({"edge" : chosen_transition, "performed_action" : chosen_action, "destination_state" : destination_state, "timestamp" : time.time()})
        
            #Transition through the chosen edge
            self.__current_state = destination_state
            self.__current_edge = chosen_transition

#TODO: add some logic to choose an edge (maybe planning techniques)    
    def choose_best_edge(self, edges):
        return edges[0]

    def __str__(self):
        return "DFAHandler:\nCurrent state: %s\n,DFA: %s" % (str(self.__current_state), str(self.dfa)) 


#DFA Post-Processing functions

#1) Remove the dummy initial state: 
#   if the DFA has a "dummy" initial state (initial state with only an outgoing edge with a true condition)
#   remove that state and set the next state as the initial one
def remove_initial_dummy_state(dfa : DFA, verbose=False):
    #Don't post-process DFAs consisting of a single state with a single loop edge
    if len(dfa.accepting_states) + len(dfa.rejecting_states) == 1:
        return

    initial_state_outgoing_edges : DFAEdge = dfa.initial_state.get_outgoing_edges()
    if len(initial_state_outgoing_edges) == 1:
        next_state = initial_state_outgoing_edges[0].to_node

        edge_literals = initial_state_outgoing_edges[0].get_literals()
        edge_actions = initial_state_outgoing_edges[0].get_actions()
        
        #If the edge hasn't got any guard literal nor action, this is a dummy state and we can prune it
        if not edge_literals and not edge_actions:
            
            assert ((dfa.initial_state.node_id in dfa.rejecting_states.keys()) or (dfa.initial_state.node_id in dfa.accepting_states.keys())) and (dfa.initial_state.node_id in dfa.states.keys())

            #Remove dummy state
            if dfa.initial_state.node_id in dfa.rejecting_states.keys():
                dfa.rejecting_states.pop(dfa.initial_state.node_id)
            else:
                dfa.accepting_states.pop(dfa.initial_state.node_id)
            dfa.states.pop(dfa.initial_state.node_id)
            
            #Set new initial state
            dfa.initial_state = next_state
            #Remove edge incoming from dummy state from the next state
            next_state.remove_edge(initial_state_outgoing_edges[0])


