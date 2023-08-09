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
from conversion import convert

def main_s2sm_original(env, result_folder, cfg):
    path = Path(__file__).parent 
    env_name = Path(env).stem
    problem_path = path / "problems"
    env_file = problem_path / env_name / "problem.yaml"
    config_file = problem_path / env_name / "config.yaml" 
    # convert to s2sm format
    convert(env,problem_path, cfg) # change due to refactor
    name, limits, Obstacles, agents, Thetas, Goals = read_problem(env_file)
    min_segs, max_segs, obs_steps = read_configuration(config_file)
    start = default_timer()
    makespan,refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0)
    end = default_timer()
    total_time = end - start
    print("Total Time = ", total_time) 
    print("Total Makespan = ", makespan)

    trajs = ref2traj(refs)
    # output to yaml file
    result, stats = {}, {}
    result["result"]=[]
    for idx in range(len(refs)):
        per_robot={}
        per_robot["states"]=[]
        segs = trajs[idx]
        for seg in segs:
            _, qref, _ = seg
            for i in range(len(qref)):
                # pos = np.array([qref[i][0], qref[i][1]])
                pos = np.array(qref[i])
                per_robot["states"].append(pos.tolist())
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
    
    return refs
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("env", help="file containing the environment (YAML)")
    parser.add_argument("result_folder", help="folder to save results")
    parser.add_argument("cfg_file", help="file containing configurations (YAML)")

    args = parser.parse_args()

    for i in range(1):
         main_s2sm_original(args.env, args.result_folder, args.cfg_file)


if __name__ == '__main__':
	main()
