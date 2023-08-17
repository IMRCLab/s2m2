import yaml
import argparse
from yaml.loader import SafeLoader
import numpy as np
from pathlib import Path
import json

def get_polytope(point, epsilon):
    A_rect = np.array([[-1,0,-(point[0]-epsilon[0])],
				       [1,0,point[0]+epsilon[0]],
				       [0,-1,-(point[1]-epsilon[1])],
				       [0,1,point[1]+epsilon[1]]])
    return A_rect.tolist()

def convert(env,env_folder,cfg):
    data_cfg = yaml.load(cfg, Loader=SafeLoader)
    epsilon = data_cfg["goal_epsilon"]
    env_name = Path(env).stem
    with open(env) as f:
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
        per_robot["k"] = [0.5]*len(robots[i]["start"]) # 2D for single integrator
        
        per_robot["type"] = robots[i]["type"]
        if per_robot["type"] == "unicycle_first_order_0":
            per_robot["size"] = 0.5 
        elif per_robot["type"] == "car_first_order_0":
            per_robot["size"] = 0.5 
        elif per_robot["type"] == "single_integrator_0":
            per_robot["size"] = 0.1
        else:
            print("Unknown robot type, manual termination!")
            raise SystemExit()


        per_robot["velocity"] = 0.5 # needs to match OMPL + VIS!
        new_format["agents"].append(per_robot)
        new_format["goals"].append(get_polytope(robots[i]["goal"], [epsilon, epsilon])) 
        new_format["starts"].append(robots[i]["start"][:2]) # position just, no orientation
    for j in range(len(obstacles)):
        new_format["obstacles"].append(get_polytope(obstacles[j]["center"], np.array(obstacles[j]["size"]) / 2))
    
    with open(Path(env_folder) / env_name / 'problem.yaml', 'w') as outfile:
        yaml.dump(new_format, outfile)   

def extract_results(env_file, models, ma_starts, ma_segs):
    # get robot types
    agent_types=[]
    with open(env_file, "r") as file:
        data = yaml.load(file, Loader=yaml.Loader)
        robots = data['robots'] 
    for i in range(len(robots)):
        agent_types.append(robots[i]["type"])   

    agent_num = len(models)
    paths = []
    for idx in range(agent_num):
        path = []
        segs = ma_segs[idx]
        start_state = ma_starts[idx]
        if(agent_types[idx] == "unicycle_first_order_0"):
            q0 = start_state + [0] # adding theta manually
            for seg in segs:
                t, qref, uref = seg
                # run the controller
                run = models[idx].run_model
                q = run(q0, t, qref, uref)
                q0 = q[-1]
                for i in range(len(q)-1):
                    path.append(q[i])
        paths.append(path)
    return paths
