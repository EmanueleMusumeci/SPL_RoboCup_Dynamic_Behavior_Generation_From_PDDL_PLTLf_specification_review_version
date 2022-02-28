from typing import List

from ltlf2dfa.parser.ltlf import LTLfParser
import lark

from bool import ValueRegistry, LiteralRegistry, Literal
from action import Action, ActionRegistry


class LTLRule:
    def __init__(self, formula_string : str, mona_dfa_out=True):
        assert isinstance(formula_string, str)

        parser = LTLfParser()
        try:
            formula = parser(formula_string)
        except lark.exceptions.UnexpectedCharacters as e:
            print("UnexpectedCharacter exception occured in LTLfParser:\n"+"-"*100)
            print(e)
            print("-"*100)
            print("HINTS:\n1) Use only lower-case characters in the formula, upper-case are reserved for LTL operators\n2) The only special character allowed is '_'")
            exit(1)
        except lark.exceptions.UnexpectedToken as e:
            print("UnexpectedToken exception occured in LTLfParser:\n"+"-"*100)
            print(e)
            print("-"*100)
            print("HINTS:\n1) Use only lower-case characters in the formula, upper-case are reserved for LTL operators\n2) You might have used a wrong syntax: use parentheses '(',')' to structure your formula\n3) You migh have used an unsuported LTL operator: check this page for syntax: http://ltlf2dfa.diag.uniroma1.it/ltlf_syntax")
            exit(1)
        except Exception as e:
            raise e

        labels = formula.find_labels()
        assert labels

        self.__literals = []
        self.__actions = []
        print(labels)
        for label in labels:
            if label in LiteralRegistry():
                self.__literals.append(label)
            elif label in ActionRegistry():
                self.__actions.append(label)
            else:
                raise AssertionError("Label '%s' is not a 'literal' nor an 'action'" % (label))

        assert self.__literals or self.__actions, "The formula contains no literals nor actions! REMEMBER: literals should have a 'literal_' prefix and actions should have an 'action_' prefix"

        self.formula = formula
    
if __name__ == "__main__":
    print("Tests: ActionRegistry creation and print")
    action_registry = ActionRegistry()
    action_registry["action_kick_ball"] = "kick"
    print(action_registry)
    ActionRegistry()["action_reach_ball"] = "reachBall"
    print(ActionRegistry())
    print("\n\n")

    print("Tests: ValueRegistry creation and print")
    value_registry = ValueRegistry()

    value_registry["distance"] = 100
    print(value_registry)

    ValueRegistry()["threshold1"] = 200
    ValueRegistry()["threshold2"] = 50
    print(ValueRegistry())
    print("\n\n")

    print("Tests: LiteralRegistry creation and print")
    literal_registry = LiteralRegistry()
    literal_registry["this_value_should_be_true"] = True
    print(literal_registry)

    LiteralRegistry()["this_value_should_be_false"] = False
    print(LiteralRegistry())
    print("\n\n")

    print("Tests: SimpleLiteral evaluation")
    print("this_value_should_be_true AND this_value_should_be_false SHOULD BE False: "+str(literal_registry["this_value_should_be_true"] and literal_registry["this_value_should_be_false"]))
    print("this_value_should_be_true OR this_value_should_be_false SHOULD BE True: "+str(literal_registry["this_value_should_be_true"] or literal_registry["this_value_should_be_false"]))
    print("\n\n")

    print("Tests: FunctionLiteral evaluation")
    def distance_lower_than_threshold1(distance, threshold1):
        return distance < threshold1

    def distance_lower_than_threshold2(distance, threshold2):
        return distance < threshold2

    LiteralRegistry().add_function_literal(distance_lower_than_threshold1)
    LiteralRegistry().add_function_literal(distance_lower_than_threshold2)
    print(LiteralRegistry())
    print("\n\n")


    print("Tests: LTLRule construction")
    ltl_formula_1_str = "G(a)"
    print("Testing formula: %s" % (ltl_formula_1_str))
    try:
        ltl_formula = LTLRule(ltl_formula_1_str)
    except AssertionError:
        print("OK")
    else:
        raise Exception("Should raise an AssertionError because we did not register literal 'a' or literal 'literal_a'")
    print("\n")
        
    ltl_formula_2_str = "G(distance_lower_than_threshold1)"
    print("Testing formula: %s" % (ltl_formula_2_str))
    try:
        ltl_formula = LTLRule(ltl_formula_2_str)
    except AssertionError:
        raise Exception("Should not raise an error")
    else:
        print("OK")
    print("\n")
        
    ltl_formula_3_str = "G(distance_lower_than_threshold1 -> a)"
    print("Testing formula: %s" % (ltl_formula_3_str))
    try:
        ltl_formula = LTLRule(ltl_formula_3_str)
    except AssertionError:
        print("OK")
    else:
        raise Exception("Should raise an AssertionError because we did not register action 'a' or action 'action_a'")
    print("\n")

    ltl_formula_3_str = "G(distance_lower_than_threshold1 -> action_a)"
    print("Testing formula: %s" % (ltl_formula_3_str))
    try:
        ltl_formula = LTLRule(ltl_formula_3_str)
    except AssertionError:
        print("OK")
    else:
        raise Exception("Should raise an AssertionError because we did not register action 'a' or action 'action_a'")
    print("\n")
        
    ltl_formula_4_str = "G(distance_lower_than_threshold1 -> kick_ball)"
    print("Testing formula: %s" % (ltl_formula_4_str))
    try:
        ltl_formula = LTLRule(ltl_formula_4_str)
    except AssertionError:
        raise Exception("Should not raise an error")
    else:
        print("OK")
    print("\n")

    ltl_formula_4_str = "G(distance_lower_than_threshold1 -> action_kick_ball)"
    print("Testing formula: %s" % (ltl_formula_4_str))
    try:
        ltl_formula = LTLRule(ltl_formula_4_str)
    except AssertionError:
        raise Exception("Should not raise an error")
    else:
        print("OK")
    print("\n")
        