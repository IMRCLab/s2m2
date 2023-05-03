from algs.decentralized import decentralized_algo
from algs.ref2traj import ref2traj
from problems.util import *
from timeit import *
from util import *

from viz.plot import *
from viz.animate import *
from viz.util import *

def test(env, problem_path, config_path):
    name, limits, Obstacles, agents, Thetas, Goals = read_problem(problem_path)
    min_segs, max_segs, obs_steps = read_configuration(config_path)

    start = default_timer()
    refs = decentralized_algo(agents, Thetas, Goals, limits, Obstacles, min_segs, max_segs, obs_steps, 0) # [[t1, px1, py1]...[tf,pxf,pyf]] for each agent. f=number of waypoints
    end = default_timer()
    print("Total Time = ", end - start)
    name = '[%s]'%(env)

    trajs = ref2traj(refs) # ma_nodes -> ma_segs, [times, qref, uref] = trajs from rej2traj
    # plot_results(agents, limits, Obstacles, Thetas, Goals, trajs, name, refs=refs)
    animate_results(agents, limits, Obstacles, Thetas, Goals, trajs, name)

    return refs

test("ground", "problems/ground/problem.yaml", "problems/ground/config.yaml")
