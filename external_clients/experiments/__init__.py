#https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
import os

dir = os.path.dirname(os.path.abspath(__file__))
modules = [os.path.splitext(_file)[0] for _file in os.listdir(dir) if not _file.startswith('__')]

experiments = []
for mod in modules:
    exec('from experiments import {}; experiments.append({})'.format(mod, mod))