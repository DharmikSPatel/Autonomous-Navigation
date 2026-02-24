import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import argparse
import utils

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['arm', 'freeBody'])
    parser.add_argument('--target', nargs='*', type=float)
    parser.add_argument('-k', type=int)
    parser.add_argument('--configs')
    # args = parser.parse_args(["--robot", "arm", "--target", "0", "0", "-k", "3", "--configs", "configs_arm.txt"])
    args = parser.parse_args()
    return args
def generate_and_write_configs(num_of_points, robot_type, fname):
    if robot_type == 'arm':
        points = np.empty((0, 2))
    elif robot_type == 'freeBody':
        points = np.empty((0, 3))

    for _ in range(num_of_points):
        if robot_type == 'arm':
            pose = utils.random_pose_thetas().T
        elif robot_type == 'freeBody':
            pose = utils.random_pose_xytheta().T
        points = np.vstack((points, pose))
    np.savetxt(fname, points)
def visulaize_k_nearest(robot_type: str, start: np.ndarray, near: np.ndarray, all_points: np.ndarray, viewAll = False):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(1, 1, 1, xlim=(-10,10), ylim=(-10, 10), aspect='equal')
    
    if robot_type == 'arm':
        utils.draw_2arm_and_frame(ax, start[np.newaxis].T)
    elif robot_type == 'freeBody':
        utils.draw_rectangle_and_frame(ax, start[np.newaxis].T)
    if viewAll:
        for o in all_points:
            if robot_type == 'arm':
                utils.draw_2arm_and_frame(ax, o[np.newaxis].T, ec='k')
            elif robot_type == 'freeBody':
                utils.draw_rectangle_and_frame(ax, o[np.newaxis].T, ec='k')

    for i,n in enumerate(near):
        if robot_type == 'arm':
            utils.draw_2arm_and_frame(ax, n[np.newaxis].T, ec='g')
        elif robot_type == 'freeBody':
            utils.draw_rectangle_and_frame(ax, n[np.newaxis].T, ec='g')
            # ax.text(n[0], n[1], "%d|%s|%s" % (i, delete_dist_freeBody(start, n), n))
    plt.grid()
    # plt.savefig(f"nn_{args.robot}_config{num}")
    plt.show()
def get_k_nearest(robot_type: str, k: int, start: np.ndarray, points: np.ndarray) -> np.ndarray:
    distances = []
    # print(start.shape)
    # print(points.shape)
    for p in points:
        if robot_type == 'arm':
            distances.append((dist_arm(start, p), p))
        elif robot_type == 'freeBody':
            distances.append((dist_freeBody(start, p), p))
    distances.sort(key=lambda d: d[0])
    near = [d[-1] for d in distances[:k]]
    return np.array(near)
def dist_arm(pose1: np.ndarray, pose2: np.ndarray) -> float:
    delta_pose = np.abs(utils.normalize_angle(pose2 - pose1))
    delta_pose = np.minimum(delta_pose, 2*np.pi - delta_pose)
    return np.sqrt(np.sum(delta_pose ** 2))
def dist_freeBody(pose1: np.ndarray, pose2: np.ndarray):
    # row vectors
    pose1[2] = utils.normalize_angle(pose1[2])
    pose2[2] = utils.normalize_angle(pose2[2])
    delta_pose = np.abs(pose2-pose1)
    # delta_pose[2] = utils.normalize_angle(delta_pose[2]) + np.pi
    # d = np.sqrt(np.sum(delta_pose ** 2))

    translation_dist = np.sqrt(np.sum(delta_pose[0:2]**2))
    rotation_dist = min(delta_pose[2], 2*np.pi - delta_pose[2])
    assert((translation_dist + rotation_dist) >= 0)

    return translation_dist + rotation_dist
# def delete_dist_freeBody(pose1: np.ndarray, pose2: np.ndarray):
#     # row vectors
#     pose1[2] = utils.normalize_angle(pose1[2])
#     pose2[2] = utils.normalize_angle(pose2[2])
#     delta_pose = np.abs(pose2-pose1)
#     # delta_pose[2] = utils.normalize_angle(delta_pose[2]) + np.pi
#     # d = np.sqrt(np.sum(delta_pose ** 2))

#     translation_dist = np.sqrt(np.sum(delta_pose[0:2]**2))
#     rotation_dist = min(delta_pose[2], 2*np.pi - delta_pose[2])
#     return (translation_dist, rotation_dist, translation_dist + rotation_dist)
if __name__ == "__main__":
    # for i in range(5):
    #     generate_and_write_configs(10, "arm", f"configs_arm_{i}.txt")
    #     generate_and_write_configs(100,"freeBody", f"configs_freeBody_{i}.txt")

    args = get_args()
    

    start = np.array(args.target)
    if args.robot == 'arm':
        points = np.loadtxt(args.configs, converters=utils.normalize_angle)
    elif args.robot == 'freeBody':
        points = np.loadtxt(args.configs, converters={2: utils.normalize_angle})
    near = get_k_nearest(args.robot, args.k, start, points)
    visulaize_k_nearest(args.robot, start, near, points, viewAll=False)
    
    
