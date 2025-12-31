import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import utils

def generate_environment(number_of_obstacles: int) -> np.ndarray:
    n = 0
    envi = np.empty((0, 5))
    while n != number_of_obstacles:
        # obstacle = [w, h, x, y, theta]
        obstacle = np.hstack((utils.random_wh(), *utils.random_pose_xytheta()))
        n+=1
        envi = np.vstack((envi, obstacle))
    return envi
def scene_to_file(env: np.ndarray, filename: str):
    np.savetxt(filename, env)
def scene_from_file(filename: str) -> np.ndarray:
    return np.loadtxt(filename , converters={4: utils.normalize_angle})

def visualize_scene(env: np.ndarray):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(1, 1, 1, xlim=(-10,10), ylim=(-10, 10), aspect='equal')
    for o in env:
        ax.add_patch(patches.Polygon(utils.rect_to_points(*o).T, ec='g', fc='None', hatch='XXXX'))
    plt.grid()
    plt.show()
def visualize_scene_diff_axes(ax, env: np.ndarray, ec='g'):
    for o in env:
        ax.add_patch(patches.Polygon(utils.rect_to_points(*o).T, ec=ec, fc='None', hatch='XXXX'))
def show_all(envs):
    fig, axs = plt.subplots(1, 5, figsize=(15, 8))
    axs = axs.flatten()
    for i, env in enumerate(envs):
        axs[i].set_xlim(-10, 10)
        axs[i].set_ylim(-10, 10)
        axs[i].set_title("Map %d" % i)
        axs[i].set_aspect('equal')
        axs[i].grid()
        for o in env:
            axs[i].add_patch(patches.Polygon(utils.rect_to_points(*o).T, ec='b', fc='None', hatch='XXXX'))   
    plt.show()
def read_all(map_names):
    envs = []
    for name in map_names:
        env = scene_from_file(name)
        envs.append(env)
    show_all(envs)
def generate_all(num_of_obstacls):
    for i, n in enumerate(num_of_obstacls):
        env = generate_environment(n)
        scene_to_file(env, 'map%d' % i)
        visualize_scene(scene_from_file('map%d' % i)) #shows all maps back to back, need to cross out window to show next
if __name__ == "__main__":
    num_of_obstacls = [2, 5, 10, 15, 20]

    # generate_all(num_of_obstacls) # comment line out if you do not want to RE-generate each time. for coding purposes
    map_names = ['map%d' % i for i in range(5)]
    read_all(map_names)