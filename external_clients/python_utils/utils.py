def tab_all_lines_in_string(string, times = 1):
    result_string = "\t"
    
    lines  = string.split("\n")
    for i, line in enumerate(lines):
        if not line:
            result_string += ("\n" if i < len(lines) else "")
        else:
            result_string += line + ("\n" + "".join(["\t"] * times) if i < len(lines) else "")
    return result_string