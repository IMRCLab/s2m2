import yaml
import argparse
from yaml.loader import SafeLoader
import numpy as np
from pathlib import Path
# python3 conversion.py path-to-our-environment.yaml
def get_polytope(point, epsilon=0.5):
    A_rect = np.array([[-1,0,-point[0]-epsilon],
				       [1,0,point[0]+epsilon],
				       [0,-1,-point[1]-epsilon],
				       [0,1,point[1]+epsilon]])
    return A_rect.tolist()

def convert(env_yaml, env_name):
    with open(env_yaml) as f:
        data = yaml.load(f, Loader=SafeLoader)
    environment = data["environment"]
    obstacles = environment['obstacles']
    limits_max = environment["max"]
    limits_min = environment["min"]
    robots = data['robots'] 
    new_format = {}
    new_format["agents"], new_format["goals"], new_format["starts"] = [], [], []
    new_format["limits"], new_format["obstacles"] = [], []

    new_format["name"] = env_name
    new_format["limits"].append([limits_min[0],limits_max[0]])
    new_format["limits"].append([limits_min[1],limits_max[1]])

    for i in range(len(robots)):
        per_robot = {}
        per_robot["k"] = [0.5, 0.5] # 2D for single integrator
        per_robot["size"] = 1.5
        per_robot["type"] = robots[i]["type"]
        per_robot["velocity"] = 1.0
        new_format["agents"].append(per_robot)
        new_format["goals"].append(get_polytope(robots[i]["goal"]))
        new_format["starts"].append(robots[i]["start"])
    for j in range(len(obstacles)):
        new_format["obstacles"].append(get_polytope(obstacles[j]["center"])) # size ?
    with open(Path().resolve() / 'problems' / env_name / 'problem.yaml', 'w') as outfile:
        yaml.dump(new_format, outfile)   



def main():
    parser = argparse.ArgumentParser(description='Convert our data format to s2s format')
    parser.add_argument('environment_yaml', help="environment.yaml file") 
    args = parser.parse_args()
    env_name = Path(args.environment_yaml).stem
    convert(args.environment_yaml, env_name)

if __name__ == "__main__":
    main()