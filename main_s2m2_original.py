from algs.decentralized import decentralized_algo
from algs.ref2traj import ref2traj
from problems.util import *
from timeit import *
from util import *
from viz.plot import *
from viz.animate import *
from viz.util import *
from pathlib import Path
import argparse
from conversion import *

def get_config_file(file, min_seg, max_seg, obs_seg):
    with open(file, 'w') as f:
        f.write('{')
        f.write('min_segs: %i, max_segs: %i, obstacle_segs: %i' % (min_seg, max_seg, obs_seg))
        f.write('}')


def main_s2sm_original(env, result_folder, timelimit, cfg):
    path = Path(__file__).parent 
    env_name = Path(env).stem
    problem_path = path / "problems"
    env_path = problem_path / env_name
    env_path.mkdir(parents=True, exist_ok=True)
    env_file =  env_path / "problem.yaml"
    config_file = env_path / "config.yaml" 
    # read configurations
    data_cfg = yaml.load(cfg, Loader=SafeLoader)
    min_seg = data_cfg["min_seg"]
    max_seg = data_cfg["max_seg"]
    obs_seg = data_cfg["obs_seg"]
    if config_file.is_file() == False:
        get_config_file(config_file, min_seg, max_seg, obs_seg) # min_segs, max_segs, obs_segs
    # convert to s2sm format
    format_to_s2m2(env,problem_path,cfg) 
    name, limits, Obstacles, agents, Thetas, Goals = read_problem(env_file)
    min_segs, max_segs, obs_steps = read_configuration(config_file)
    start = default_timer()
    makespan,refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0, int(timelimit))
    end = default_timer()
    total_time = end - start
    print("Total Time = ", total_time) 
    print("Total Makespan = ", makespan)

    trajs = ref2traj(refs)
    animate_results(agents, limits, Obstacles, Thetas, Goals, trajs, result_folder)

    true_trajs, true_actions = extract_results(env,agents,Thetas,trajs, result_folder)
    result, stats = {}, {}
    result["result"]=[]
    for idx in range(len(refs)):
        per_robot={}
        per_robot["states"]=[]
        per_robot["actions"]=[]
        states = true_trajs[idx]
        actions = true_actions[idx]
        for state in states:
            true_q = np.array(state)
            per_robot["states"].append(true_q.tolist())
        for action in actions:
            true_u = np.array(action)
            per_robot["actions"].append(true_u.tolist())
        result["result"].append(per_robot)

    with open(Path(result_folder) / 'result_s2sm.yaml', 'w') as outfile:
        yaml.dump(result, outfile)   
        
    stats["stats"]=[]
    stats["stats"]
    solution = {}
    solution["t"]=total_time
    solution["cost"]=makespan
    stats["stats"].append(solution)

    with open(Path(result_folder) / 'stats.yaml', 'w') as outfile:
        yaml.dump(stats, outfile) 

    # for the reference path
    ref_result = {}
    ref_result["result"]=[]
    for idx in range(len(refs)):
        per_robot={}
        per_robot["states"],per_robot["actions"]=[],[]
        segs = trajs[idx]
        for seg in segs:
            _, qref, uref = seg
            for i in range(len(qref)):
                pos = np.array(qref[i])
                u = np.array(uref[i])
                per_robot["states"].append(pos.tolist())
                per_robot["actions"].append(u.tolist())
        ref_result["result"].append(per_robot)

    with open(Path(result_folder) / 'reference_result_s2sm.yaml', 'w') as ref_outfile:
        yaml.dump(ref_result, ref_outfile)  

    return refs
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("env", help="file containing the environment (YAML)")
    parser.add_argument("result_folder", help="folder to save results")
    parser.add_argument("timelimit", help="timelimit to solve the problem")
    parser.add_argument("cfg_file", help="file containing configurations (YAML)")

    args = parser.parse_args()

    for i in range(1):
         main_s2sm_original(args.env, args.result_folder, args.timelimit, args.cfg_file)


if __name__ == '__main__':
	main()
