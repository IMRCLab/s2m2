import yaml
import argparse
from yaml.loader import SafeLoader
import numpy as np
from pathlib import Path
from math import *

def get_polytope(point, epsilon):
    A_rect = np.array([[-1,0,-(point[0]-epsilon[0])],
				       [1,0,point[0]+epsilon[0]],
				       [0,-1,-(point[1]-epsilon[1])],
				       [0,1,point[1]+epsilon[1]]])
    return A_rect.tolist()

def format_to_s2m2(env,env_folder,epsilon):
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
            per_robot["k"] = [2.0, 2.0, 1.0]
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

def extract_output(models, ma_starts, ma_segs):
    # get robot types
    agent_num = len(models)
    paths, actions = [], []
    for idx in range(agent_num):
        segs = ma_segs[idx]
        start_state = ma_starts[idx]
        q0 = start_state + [0] # read from .yaml
        q = [q0] 
        u = [] 
        for seg in segs:
            u0 = [0, 0]
            t, qref, uref = seg 
            for i in range(0, len(t)):
                t_step = [t[i-1], t[i]]
                qref_i, uref_i = qref[i], uref[i]
                run = models[idx].run_model_on_result
                q0, u0 = run(q0, u0, t_step, qref_i, uref_i) # propagated
                if i >= 1: # avoid dup;icate of the start state
                    q.append(q0) # sequence of states
                    u.append(u0)
                i += 1
        
        paths.append(q)
        actions.append(u)   
    # compare last point and final
    print(sqrt((q[-1][0]-qref[-1][0])**2 + (q[-1][1]-qref[-1][1])**2))    
    return paths, actions

def extract_results(env_file, models, ma_starts, ma_segs):
    # get robot types
    agent_types=[]
    with open(env_file, "r") as file:
        data = yaml.load(file, Loader=yaml.Loader)
        robots = data['robots'] 
    for i in range(len(robots)):
        agent_types.append(robots[i]["type"])   

    agent_num = len(models)
    paths, actions = [], []
    for idx in range(agent_num):
        segs = ma_segs[idx]
        start_state = ma_starts[idx]
        if idx == 1:
            q0 = start_state + [3.14] # read from .yaml
        else:
            q0 = start_state + [0] # read from .yaml
        qs = [q0] 
        us = [] 
        q = q0
        if(agent_types[idx] == "unicycle_first_order_0"):
            for seg in segs:
                t, qref, uref = seg 
                for i in range(0, len(t)):
                    # t_step = [0.0, 0.1] #[t[i-1], t[i]]
                    qref_i, uref_i = qref[i], uref[i]
                    # run = models[idx].run_model_on_result
                    # q0, u0 = run(q0, u0, t_step, qref_i, uref_i) # propagated

                    u = models[idx].controller(q, qref_i, uref_i)
                    # action saturation
                    u = np.clip(u, [-0.5, -0.5], [0.5, 0.5])
                    # propagate using simple Euler integration
                    q = np.asarray(q) + np.asarray(models[idx].model(q, 0, u)) * 0.1
                    # store result
                    qs.append(q) # sequence of states
                    us.append(u)

        paths.append(qs)
        actions.append(us)
        
            # print(sqrt((q0[0]-qref[-1][0])**2 + (q0[1]-qref[-1][1])**2))
            # t_n = t[-1]
            # if ((sqrt((q0[0]-qref[-1][0])**2 + (q0[1]-qref[-1][1])**2)) > 0.1):
            #     for j in range(500):
            #         t_step = [t_n, t_n + 0.01]
            #         qref_i, uref_i = qref[-1], uref[-1]
            #         q0, u0 = run(q0, u0, t_step, qref_i, uref_i) # propagated
            #         print(q0)
            #         t_n += 0.01
            #         print(sqrt((q0[0]-qref[-1][0])**2 + (q0[1]-qref[-1][1])**2))
    return paths, actions
