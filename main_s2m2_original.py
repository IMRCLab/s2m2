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
    if config_file.is_file() == False:
        get_config_file(config_file, 2, 10, 10) # min_segs, max_segs, obs_segs
    # convert to s2sm format
    convert(env,problem_path, cfg) # change due to refactor
    name, limits, Obstacles, agents, Thetas, Goals = read_problem(env_file)
    min_segs, max_segs, obs_steps = read_configuration(config_file)
    start = default_timer()
    makespan,refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0, int(timelimit))
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
    parser.add_argument("timelimit", help="timelimit to solve the problem")
    parser.add_argument("cfg_file", help="file containing configurations (YAML)")

    args = parser.parse_args()

    for i in range(1):
         main_s2sm_original(args.env, args.result_folder, args.timelimit, args.cfg_file)


if __name__ == '__main__':
	main()
