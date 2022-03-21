import math 
import difflib

def find_similar_strings(string, list_of_strings):
    return difflib.get_close_matches(string, list_of_strings)

def tab_all_lines_in_string(string, times = 1):
    result_string = "\t"
    
    lines  = string.split("\n")
    for i, line in enumerate(lines):
        if not line:
            result_string += ("\n" if i < len(lines) else "")
        else:
            result_string += line + ("\n" + "".join(["\t"] * times) if i < len(lines) else "")
    return result_string


#Add function that computes distance of supporter from striker to ValueRegistry
def distance(point1, point2):
    assert isinstance(point1, tuple)
    assert len(point1) == 2
    assert isinstance(point1[0], float) or isinstance(point1[0], int)
    assert isinstance(point1[1], float) or isinstance(point1[1], int)

    assert isinstance(point2, tuple)
    assert len(point2) == 2
    assert isinstance(point2[0], float) or isinstance(point2[0], int)
    assert isinstance(point2[1], float) or isinstance(point2[1], int)

    return math.sqrt(pow(point1[0]-point2[0], 2) + pow(point1[1]-point2[1], 2))