import yaml
import argparse
from yaml.loader import SafeLoader
import numpy as np
from pathlib import Path
from math import *

def get_polytope(point, b):
    A_rect = np.array([[-1,0,-(point[0]-b[0])],
				       [1,0,point[0]+b[0]],
				       [0,-1,-(point[1]-b[1])],
				       [0,1,point[1]+b[1]]])
    return A_rect.tolist()

def format_to_s2m2(env,env_folder,cfg_file):
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
    x_max = limits_max[0]
    y_max = limits_max[1]
    x_min = limits_min[0]
    y_min = limits_min[1]
    new_format["limits"].append([x_min,x_max])
    new_format["limits"].append([y_min,y_max])
    # read algorithms.yaml
    data_cfg = yaml.load(cfg_file, Loader=SafeLoader)
    radius = data_cfg["radius"]
    epsilon = data_cfg["goal_epsilon"]
    for i in range(len(robots)):
        per_robot = {}
        per_robot["k"] = [0.5]*len(robots[i]["start"]) 
        
        per_robot["type"] = robots[i]["type"]
        per_robot["velocity"] = 0.5 
        if per_robot["type"] == "unicycle_first_order_0_sphere":
            per_robot["size"] = radius
            per_robot["k"] = data_cfg["k"] 
            per_robot["velocity"] = data_cfg["velocity"]
            per_robot["bloating"] = data_cfg["bloating"]
        elif per_robot["type"] == "car_first_order_0":
            per_robot["size"] = 0.5 
        elif per_robot["type"] == "single_integrator_0":
            per_robot["size"] = 0.1
        else:
            print("Unknown robot type, manual termination!")
            raise SystemExit()

        new_format["agents"].append(per_robot)
        new_format["goals"].append(get_polytope(robots[i]["goal"], [epsilon, epsilon])) 
        new_format["starts"].append(robots[i]["start"][:2]) # position just, no orientation
    for j in range(len(obstacles)):
        new_format["obstacles"].append(get_polytope(obstacles[j]["center"], np.array(obstacles[j]["size"]) / 2))
   
    # add four obstacle to have the workspace limit
    
    new_format["obstacles"].append(get_polytope([x_min-radius-0.1,(y_max-y_min)/2], [0.2,(y_max-y_min)/2]))
    new_format["obstacles"].append(get_polytope([x_max+radius+0.1,(y_max-y_min)/2], [0.2,(y_max-y_min)/2]))
    new_format["obstacles"].append(get_polytope([(x_max-x_min)/2,y_min-radius-0.1], [(x_max-x_min)/2,0.2]))
    new_format["obstacles"].append(get_polytope([(x_max-x_min)/2,y_max+radius+0.1], [(x_max-x_min)/2,0.2]))
    
    with open(Path(env_folder) / env_name / 'problem.yaml', 'w') as outfile:
        yaml.dump(new_format, outfile)   

def extract_results(env_file, models, ma_starts, ma_segs, result_folder):
    # get robot types
    agent_types=[]
    agent_start_angles=[]
    agent_goal_angles=[]
    with open(env_file, "r") as file:
        data = yaml.load(file, Loader=yaml.Loader)
        robots = data['robots'] 
    for i in range(len(robots)):
        agent_types.append(robots[i]["type"])  
        agent_start_angles.append(robots[i]["start"][-1])  # unicycle
        agent_goal_angles.append(robots[i]["goal"][-1])  # unicycle

    agent_num = len(models)
    paths, actions = [], []
    for idx in range(agent_num):
        segs = ma_segs[idx]
        start_state = ma_starts[idx]
        q0 = start_state + [agent_start_angles[idx]] 
        qs = [q0]
        qrs = [segs[0][1][0]]
        ts = [0]
        us = []
        urs = []
        q = q0
        if(agent_types[idx] == "unicycle_first_order_0_sphere"):
            for seg in segs:
                t, qref, uref = seg
                qref = np.array(qref)
                uref = np.array(uref)

                sampled_t = np.arange(t[0], t[-1], 0.1)
                qref_sampled = np.vstack([
                    np.interp(sampled_t, t, qref[:,0]),
                    np.interp(sampled_t, t, qref[:,1]),
                    np.interp(sampled_t, t, qref[:,2])
                ]).T
                uref_sampled = np.vstack([
                    np.interp(sampled_t, t, uref[:,0]),
                    np.interp(sampled_t, t, uref[:,1])
                ]).T
                # print(qref_sampled)
                
                # print(t)
                for t, qref_i, uref_i in zip(sampled_t, qref_sampled, uref_sampled):
                    # t_step = [0.0, 0.1] #[t[i-1], t[i]]
                    # qref_i, uref_i = qref[i], uref[i]
                    # run = models[idx].run_model_on_result
                    # q0, u0 = run(q0, u0, t_step, qref_i, uref_i) # propagated
                    # print(qref_i, uref_i)
                    u = models[idx].controller(q, qref_i, uref_i)
                    # action saturation
                    u = np.clip(u, [-0.5, -2.0], [0.5, 2.0])
                    # propagate using simple Euler integration
                    q = np.asarray(q) + np.asarray(models[idx].model(q, 0, u)) * 0.1
                    # store result
                    qs.append(q) # sequence of states
                    qrs.append(qref_i)
                    urs.append(uref_i)
                    ts.append(t)
                    us.append(u)
                    # print(qref_i, q)

            # print(np.linalg.norm(qref_i - q))
            qref_i[2] = agent_goal_angles[idx]
            while True:
                # print(t, np.linalg.norm(qs[-2] - qs[-1]))
                t += 0.1
                u = models[idx].controller(q, qref_i, uref_i)
                print(t, idx, u)
                if np.linalg.norm(u) < 0.1:
                    break
                # action saturation
                u = np.clip(u, [-0.5, -2.0], [0.5, 2.0])
                # propagate using simple Euler integration
                q = np.asarray(q) + np.asarray(models[idx].model(q, 0, u)) * 0.1
                # store result
                qs.append(q) # sequence of states
                qrs.append(qref_i)
                urs.append(uref_i)
                ts.append(t)
                us.append(u)

            import matplotlib.pyplot as plt

            # Data for plotting

            fig, ax = plt.subplots(4,1)
            for i in range(3):
                ax[i].plot(ts, np.asarray(qrs)[:,i], color='b', linestyle='dashed')
                ax[i].plot(ts, np.asarray(qs)[:,i], color='b')

            ax[3].plot(ts[1:], np.asarray(urs)[:,0], color='b', linestyle='dashed')
            ax[3].plot(ts[1:], np.asarray(us)[:,0], color='b')
            ax[3].plot(ts[1:], np.asarray(urs)[:,1], color='g', linestyle='dashed')
            ax[3].plot(ts[1:], np.asarray(us)[:,1], color='g')


            print(result_folder)
            fig.savefig(Path(result_folder) / "ctrl{}.png".format(idx))
            # plt.show()

        paths.append(qs)
        actions.append(us)
        
    return paths, actions
