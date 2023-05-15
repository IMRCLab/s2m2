from algs.decentralized import decentralized_algo
from algs.ref2traj import ref2traj
from problems.util import *
from timeit import *
from util import *
from viz.plot import *
from viz.animate import *
from viz.util import *
from pathlib import Path

def test(env, problem_path, config_path):
    name, limits, Obstacles, agents, Thetas, Goals = read_problem(problem_path)
    min_segs, max_segs, obs_steps = read_configuration(config_path)



    start = default_timer()
    makespan,refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0)
    end = default_timer()
    total_time = end - start
    print("Total Time = ", total_time)
    print("Makespan = ", makespan)
    # name = '[%s]'%(env)

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
                pos = np.array([qref[i][0], qref[i][1]])
                per_robot["states"].append(pos.tolist())
        result["result"].append(per_robot)

    with open(Path().resolve() / 'results' / name / 'result_s2sm.yaml', 'w') as outfile:
        yaml.dump(result, outfile)   
    
    stats["stats"]={}
    stats["stats"]["time"]=total_time
    stats["stats"]["cost"]=makespan

    with open(Path().resolve() / 'results' / name / 'stats.yaml', 'w') as outfile:
        yaml.dump(stats, outfile) 
    
    # plot_results(agents, limits, Obstacles, Thetas, Goals, trajs, name, refs=refs)
    # animate_results(agents, limits, Obstacles, Thetas, Goals, trajs, name)

    return refs

test("wall", "problems/wall/problem.yaml", "problems/wall/config.yaml")
