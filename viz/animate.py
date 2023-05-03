import matplotlib.animation as animation
from viz.util import *
import os
from models.agent import *

def animate_results(models,  limits, obstacles, Thetas, goals, ma_segs, name):
    agent_num = len(models)
    dim = len(limits)
    fig, axes = plot_env(limits, obstacles)
    plot_goals(goals)

    interval = 20
    total_frames = 500
    total_time = max([ma_segs[idx][-1][0][-1] for idx in range(agent_num)])

    paths = extract_paths(models, Thetas, ma_segs) # runs controller here
    ref_patches = []
    for idx in range(agent_num):
        ref_x, ref_y, times = paths[idx]
        ref_patch = plt.Circle((ref_x[0], ref_y[0]), models[idx].size, fc='red', alpha = 0.7)
        ref_patches.append(ref_patch)

    def init():
        for idx in range(agent_num):
            ref_x, ref_y, times = paths[idx] # taking just x,y
            ref_patches[idx].center = (ref_x[0], ref_y[0])

        for patch in ref_patches: axes.add_patch(patch)
        return ref_patches
    # animate
    tpf = total_time / total_frames

    def animate(f):
        ref = []
        for idx in range(agent_num):
            ref_x, ref_y, times = paths[idx]
            step = 0
            while (step < len(times) - 1) and (times[step] < tpf * f):
                step = step + 1
            ref_patches[idx].center = (ref_x[step], ref_y[step])
            if step == len(ref_x) - 1: error = models[idx].size
            else: error  = (models[idx].size + models[idx].bloating(step))
        return ref_patches


    ani = animation.FuncAnimation(fig, animate, frames = total_frames, init_func=init,
                                    blit=True, interval = interval)

    path = os.path.abspath("results/%s.mp4" % (name))
    plt.show()
