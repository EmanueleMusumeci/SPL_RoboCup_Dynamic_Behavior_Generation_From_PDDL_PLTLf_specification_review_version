import enum
from posixpath import split
from typing import Dict, List

from click import command
from regex import P

TEMPLATE_KEYWORD_TOKEN = "KEYWORD"
TEMPLATE_PARAMETER_TOKEN = "PARAMETER"

def get_available_constraint_templates():
    return {
        "never_something" : {
            "command_template" : ["KEYWORD_0", "PARAMETER_0"],
            "keywords" : {
                "KEYWORD_0" : ["never", "avoid", "not once"]
            },
            "formula_template" : ["H(!", "PARAMETER_0", ")"],
        },
        "something_at_least_once" : {
            "command_template" : ["KEYWORD_0", "PARAMETER_0"],
            "keywords" : {
                "KEYWORD_0" : ["at least once", "once", "sometimes"]
            },
            "formula_template" : ["O(", "PARAMETER_0", ")"]
        },
        "something_at_least_once then something else" : {
            "command_template" : ["KEYWORD_0", "PARAMETER_0", "KEYWORD_1", "PARAMETER_1"],
            "keywords" : {
                "KEYWORD_0" : ["if at least once", "if once", "if sometimes"],
                "KEYWORD_1" : ["then", "and then", "finally"]
            },
            "formula_template" : ["O(", "PARAMETER_1", ") && YO(", "PARAMETER_0", ")"]
        },
        "always_something" : {
            "command_template" : ["KEYWORD_0", "PARAMETER_0"],
            "keywords" : {
                "KEYWORD_0" : ["historically", "always"]
            },
            "formula_template" : ["H(", "PARAMETER_0", ")"],
        },
    }

#Textual matching function (for now it's just simple string equality)
def are_matching(string1, string2):
    string1 = ("").join(string1)
    string2 = ("").join(string2)
    return string1.lower() == string2.lower()

def find_next_keyword(starting_position_in_input_string, input_tokens, template_data, starting_position_in_template_structure = 0):
    #Find the keyword and its position
    for structure_token_position, structure_token in enumerate(template_data["command_template"][starting_position_in_template_structure:]):
        if structure_token.startswith(TEMPLATE_KEYWORD_TOKEN):
            #If it is a keyword check that the input string contains at some point one of the aliases for that keyword
            for input_string_token_position, input_string_token in enumerate(input_tokens[starting_position_in_input_string:]):
                #If a KEYWORD appears in the template structure, it should also appear in the "keywords" dictionary
                assert structure_token in template_data["keywords"], "Wrong constraint template_data structure"
                for alias in template_data["keywords"][structure_token]:
                    split_alias = alias.split(" ")
                    #print(split_alias)
                    #If there is no more room in the input string for the split_alias, return with null result
                    if starting_position_in_input_string + input_string_token_position + len(split_alias) > len(input_tokens):
                        #print("OUT OF STRING BOUNDS")
                        continue
                    
                    #Given a possible keyword split_alias of length M, collect the next M words of input string
                    selected_input_words = input_tokens[starting_position_in_input_string + input_string_token_position : starting_position_in_input_string + input_string_token_position + len(split_alias)]
                    if are_matching(selected_input_words, split_alias):
                        return alias, starting_position_in_template_structure+structure_token_position, starting_position_in_input_string + input_string_token_position, len(split_alias)

    
    #print("NEXT KEYWORD NOT FOUND")
    return None, -1, -1, -1

def get_next_parameter(starting_position_in_input_string, input_tokens, template_data, starting_position_in_template_structure = 0):
    assert starting_position_in_input_string < len(input_tokens), "In a template structure a parameter always follows a keyword"
    assert starting_position_in_template_structure < len(template_data["command_template"]), "In a template structure a parameter always follows a keyword"
    position_in_input_string = starting_position_in_input_string
    #A parameter always follows a keyword: we want to know how many tokens in the input string we want to store for that parameter
        # 1) We look for the next keyword
        # 2) We store all tokens in the input string between the current position and the next keyword position in the input string
    
    assert template_data["command_template"][starting_position_in_template_structure].startswith(TEMPLATE_PARAMETER_TOKEN), "In a command the template a parameter always follows a keyword"
    
    next_keyword, next_keyword_position_in_structure, alias_position_in_string, alias_length = find_next_keyword(position_in_input_string, input_tokens, template_data, starting_position_in_template_structure = starting_position_in_template_structure)
    if next_keyword is not None:
        param_length = alias_position_in_string  - position_in_input_string
        return input_tokens[position_in_input_string : alias_position_in_string], position_in_input_string, param_length

    return input_tokens[position_in_input_string : ], position_in_input_string, len(input_tokens) - position_in_input_string
            
def replace_parameters(formula_template, parameters):
    formula_result = ""
    for token in formula_template:
        if token.startswith(TEMPLATE_PARAMETER_TOKEN):
            assert token in parameters.keys(), "Parameter token was not collected in collected parameters"
            formula_result+=parameters[token]
        else:
            formula_result+=token
    
    return formula_result

#All templates start with a keyword and then an alternating sequence of parameter and optionally another keyword and so on
def match_string_with_constraint_template(input_string : str, conditionable_predicates : str):
    split_string = [word for word in input_string.split(" ")]
    
    found_template = None

    #Find the correct command template by matching the first token
    for template, template_data in get_available_constraint_templates().items():
        #Assert that the template structure is correct (starts with a keyword)
        assert template_data["command_template"][0] in template_data["keywords"].keys(), "Constraint template '"+template+"' should start with a keyword!"

        keyword, keyword_position_in_structure, alias_position_in_string, alias_length = find_next_keyword(0, split_string, template_data)

        if keyword is not None and alias_position_in_string == 0:
            #print("FOUND TEMPLATE")
            break
        
    
    assert keyword is not None, "Could not match first keyword in any template"


    template_parameters = {}

    current_position_in_string = alias_position_in_string + alias_length
    current_position_in_structure = keyword_position_in_structure + 1
    

    while keyword is not None:
        #Collect parameters:
        # 1) Find the next parameter by
        #   1.1) Looking for the next keyword or the end of the template structure
        #   1.2) Collecting all string words in between the two keywords (or between the first keyword and the end of the string)
        # 2) Associating the collected parameter to its placeholder name in the template structure
        parameter_list, parameter_position_in_string, parameter_length = get_next_parameter(current_position_in_string, split_string, template_data, starting_position_in_template_structure = current_position_in_structure)

        assert parameter_list is not None, "A keyword is always followed by a parameter"

        #Store found parameter
        template_parameters[template_data["command_template"][current_position_in_structure]] = parameter_list

        current_position_in_structure += 1
        current_position_in_string = parameter_position_in_string + parameter_length

        #Find next keyword (if there is any)
        keyword, keyword_position_in_structure, alias_position_in_string, alias_length = find_next_keyword(current_position_in_string, split_string, template_data, starting_position_in_template_structure = current_position_in_structure)

        if keyword is not None:
            current_position_in_structure = keyword_position_in_structure + 1
            current_position_in_string = alias_position_in_string + alias_length

        if current_position_in_structure > len(template_data["command_template"]):
            break
    
    assert current_position_in_structure >= len(template_data["command_template"]), "Wrong template structure (leading keyword)"

    #Check that all parameters are in the set of conditionable predicates
    for teamplate_parameter_keyword, template_parameter in template_parameters.items():
        if isinstance(template_parameter, list):
            if template_parameter[0] not in conditionable_predicates:
                assert ("_").join(template_parameter) in conditionable_predicates, "Neither '"+template_parameter[0]+"' nor '"+("_").join(template_parameter[0])+"' are conditionable (conditionable predicates: "+str(conditionable_predicates)+")"
            template_parameters[teamplate_parameter_keyword] = ("_").join(template_parameter)
        else:
            assert template_parameter in conditionable_predicates, "Parameter '"+str(template_parameter)+"' is not a conditionable predicate (conditionable predicates: '"+str(conditionable_predicates)+"')"

    #Match the parameters
    constraint_formula = replace_parameters(template_data["formula_template"], template_parameters)
    return constraint_formula


def ask_constraints_for_role(role : str, current_goal : str, conditionable_predicates : List[str]):
    add_new_constraint = True
    constraint_ok = False
    constraints = []
    while not constraint_ok and add_new_constraint:
        input_string = input("Insert a single constraint for role %s with goal '%s' (conditionable predicates %s) and press ENTER. Press ENTER with an empty constraint to SKIP:\n" % (role, current_goal, str(conditionable_predicates)))
        if input_string.strip() == "":
            print("\tNo constraint inserted.\n")
            return []
        else:
            try:
                constraint = match_string_with_constraint_template(input_string, conditionable_predicates)
            except AssertionError as e:
                print("String %s could not be recognized due to error %s" % (input_string, str(e)))
            else:
                print("Resulting constraint:\n\t%s" % (constraint))
        
        constraint_ok_response = input("Accept constraint? [Y/N]").lower()
        while constraint_ok_response.lower() not in ["y","n", "yes", "no"]:
            constraint_ok_response = input("Accept constraint? [Y/N]")
        
        constraint_ok = (True if constraint_ok_response in ["y", "YES"] else False)
        
        if constraint_ok:
            constraints.append(constraint)
        else:
            continue

        add_new_constraint_response = input("Add new constraint? [Y/N]").lower()
        while add_new_constraint_response.lower() not in ["y","n", "yes", "no"]:
            add_new_constraint_response = input("Add new constraint? [Y/N]")
        
        add_new_constraint = (True if add_new_constraint_response in ["y", "YES"] else False)

    return constraints

def ask_additional_constraints(role_to_generation_data : Dict[str, Dict[str, str]]):
    additional_constraints = {}
    for role in role_to_generation_data.keys():
        
        #Check that all conditionable predicates are lowercase (the PLTLf goal compiler accepts only lowercase predicates)
        for predicate in role_to_generation_data[role]["constrainable_predicates"]:
            assert predicate == predicate.lower(), "All conditionable predicates have to be lowercase (the PLTLf goal compiler accepts only lowercase predicates): predicate '"+predicate+"' is not."

        additional_constraint_for_this_role = ask_constraints_for_role(role, role_to_generation_data[role]["goal"], role_to_generation_data[role]["constrainable_predicates"])
        

        if additional_constraint_for_this_role is not None:
            additional_constraints[role] = additional_constraint_for_this_role
    return additional_constraints

if __name__ == "__main__":
    #!O(.)
    print("Using input string 'AVOID me' we should obtain formula '!O(me). Result:")
    result = match_string_with_constraint_template("AVOID me", conditionable_predicates = ["me"])
    print(result)
    assert result == "!O(me)", "Wrong formula"

    print("Using input string 'AVOID me' we should obtain formula '!O(you). Result:")
    try:
        result = match_string_with_constraint_template("AVOID me", conditionable_predicates = [])
    except AssertionError:
        print("OK")
    else:
        print("An AssertionError should have been raised")
        exit(1)
    
    print("Using input string 'avoid me and you' we should obtain formula '!O(me_and_you). Result:")
    result = match_string_with_constraint_template("avoid me and you", conditionable_predicates = ["me_and_you"])
    print(result)
    assert result == "!O(me_and_you)", "Wrong formula"
    
    print("Using input string 'avoid me and you' we should obtain formula '!O(me_and_you). Result:")
    try:
        result = match_string_with_constraint_template("AVOID me", conditionable_predicates = [""])
    except AssertionError:
        print("OK")
    else:
        print("An AssertionError should have been raised")
        exit(1)

    print("Using input string 'never me and you' we should obtain formula '!O(me_and_you). Result:")
    result = match_string_with_constraint_template("never me and you", conditionable_predicates = ["me_and_you"])
    print(result)
    assert result == "!O(me_and_you)", "Wrong formula"
    
    #O(.)
    print("Using input string 'at least once me' we should obtain formula 'O(me). Result:")
    result = match_string_with_constraint_template("at least once me", conditionable_predicates = ["me"])
    print(result)
    assert result == "O(me)", "Wrong formula"
    
    print("Using input string 'at least once me and you' we should obtain formula 'O(me_and_you). Result:")
    result = match_string_with_constraint_template("at least once me and you", conditionable_predicates = ["me_and_you"])
    print(result)
    assert result == "O(me_and_you)", "Wrong formula"
    
    print("Using input string 'sometimes me' we should obtain formula 'O(me). Result:")
    result = match_string_with_constraint_template("sometimes me", conditionable_predicates = ["me"])
    print(result)
    assert result == "O(me)", "Wrong formula"
    
    print("Using input string 'sometimes me and you' we should obtain formula 'O(me_and_you). Result:")
    result = match_string_with_constraint_template("sometimes me and you", conditionable_predicates = ["me_and_you"])
    print(result)
    assert result == "O(me_and_you)", "Wrong formula"

    #H(.)
    print("Using input string 'always me' we should obtain formula 'H(me). Result:")
    result = match_string_with_constraint_template("always me", conditionable_predicates = ["me"])
    print(result)
    assert result == "H(me)", "Wrong formula"

    print("Using input string 'historically me' we should obtain formula 'H(me). Result:")
    result = match_string_with_constraint_template("historically me", conditionable_predicates = ["me"])
    print(result)
    assert result == "H(me)", "Wrong formula"

    print("Using input string 'historically me and you' we should obtain formula 'H(me_and_you). Result:")
    result = match_string_with_constraint_template("historically me and you", conditionable_predicates = ["me_and_you"])
    print(result)
    assert result == "H(me_and_you)", "Wrong formula"

    #O(.) && YO(.)
    print("Using input string 'if at least once me then still me' we should obtain formula 'O(still_me) && YO(me)'. Result:")
    result = match_string_with_constraint_template("if at least once me then still me", conditionable_predicates = ["still_me", "me"])
    print(result)
    assert result == "O(still_me) && YO(me)", "Wrong formula"

    print("Using input string 'if at least once me and you then still me and you' we should obtain formula 'O(still_me_and_you) && YO(me_and_you)'. Result:")
    result = match_string_with_constraint_template("if at least once me and you then still me and you", conditionable_predicates = ["still_me_and_you", "me_and_you"])
    print(result)
    assert result == "O(still_me_and_you) && YO(me_and_you)", "Wrong formula"

