import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
import argparse
import utils
import component_1, nearest_neighbors, collision_checking
from utils import Graph
import time
EXPORT = False
ITER_TESTING = 10
PART_6_TESTING = False
RUN, K = 500 if PART_6_TESTING else 5000, 6
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['arm', 'freeBody'])
    parser.add_argument('--start', nargs='*', type=float)
    parser.add_argument('--goal', nargs='*', type=float)
    parser.add_argument('--map')
    # args = parser.parse_args(["--robot", "arm", "--start", "0", "0", "--goal", "3.93", "-3.14", "--map", "map4"])
    # args = parser.parse_args(["--robot", "freeBody", "--start", "0", "0", "0", "--goal", "1", "1.5", "2.0", "--map", "map0"])
    args = parser.parse_args()
    return args

def make_prm(obstacles, robot_type, rand_pose, near, collisonOccurs, localPlanner, dist):
    # graph = dict[pose->set of all if its edges that go to next node]
    graph = Graph()
    for _ in range(RUN):
        if _ % 100 == 0:
            print("Node", _)
        pose_rand = rand_pose()
        while collisonOccurs(pose_rand, obstacles)[0]:
            pose_rand = rand_pose()
        k_nearest = near(robot_type, K, pose_rand.flatten(), graph.get_vertices())
        graph.add_vertex(pose_rand)
        for pn in k_nearest:
            pose_near = pn[:, np.newaxis]
            path = localPlanner(pose_rand, pose_near, dist)
            if all([not collisonOccurs(n, obstacles)[0] for n in path]):
                graph.add_edge(pose_rand, pose_near)
    return graph    

def query_prm(graph,obstacles, robot_type, start, goal, near, collisonOccurs, localPlanner, dist):
    # connect start and goal to graph to the nearest node that it can get to w/o collison
    for pose in [start, goal]:
        all_ranked = near(robot_type, K, pose.flatten(), graph.get_vertices())
        connected = False
        for k, pn in enumerate(all_ranked):
            # print(f"{k}th nearst node checking")
            pose_near = pn[:, np.newaxis]
            path = localPlanner(pose[:, np.newaxis], pose_near, dist)
            if all([not collisonOccurs(n, obstacles)[0] for n in path]):
                graph.add_edge(pose, pose_near)
                connected = True
            # else:
            #     # print(f"\tNOT connected")
        if not connected:
            # then it means that no edge was found
            # therefor this else block is called
            print("Path not Found because start or goal could not be connected")
            return None, float('inf')
    
    # run A* on the graph
    print("Trying to Find Path")
    path, cost = utils.A_star(graph, dist, start, goal)
    if path:
        print("Path Found of len", len(path))
    else:
        print("Path not Found")
    return path, cost




def main(args, animate=False):
    obstacles = component_1.scene_from_file(args.map)

    if PART_6_TESTING:
        start_time = time.process_time()
    if args.robot == 'freeBody':
        graph = make_prm(obstacles, 
                        args.robot,
                        utils.random_pose_xytheta, 
                        nearest_neighbors.get_k_nearest, 
                        collision_checking.check_col_freeBody,
                        utils.interpolate_freeBody,
                        nearest_neighbors.dist_freeBody)
    else:
        graph = make_prm(obstacles, 
                        args.robot,
                        utils.random_pose_thetas, 
                        nearest_neighbors.get_k_nearest, 
                        collision_checking.check_col_arm,
                        utils.interpolate_arm,
                        nearest_neighbors.dist_arm)
    start = np.array(args.start)
    goal = np.array(args.goal)
    if args.robot == 'freeBody':
        start[2] = utils.normalize_angle(start[2])
        goal[2] = utils.normalize_angle(goal[2])
    else:
        start = utils.normalize_angle(start)
        goal = utils.normalize_angle(goal)
    if args.robot == 'freeBody':
        path, cost = query_prm(graph, obstacles,
                        args.robot,
                        start, 
                        goal,
                        nearest_neighbors.get_k_nearest, 
                        collision_checking.check_col_freeBody,
                        utils.interpolate_freeBody,
                        nearest_neighbors.dist_freeBody)
    else:
        path, cost = query_prm(graph , obstacles,
                        args.robot,
                        start, 
                        goal,
                        nearest_neighbors.get_k_nearest, 
                        collision_checking.check_col_arm,
                        utils.interpolate_arm,
                        nearest_neighbors.dist_arm)
    if PART_6_TESTING:
        end_time = time.process_time()

    path_found = True
    if path is None:
        print("No Path Found")
        path_found = False
        if not PART_6_TESTING or EXPORT:
            exit()

            
    path = np.array(path).T  #DoF x len(path). each col vector is a pose
    # print("Len Of Path", path.shape[1])
    
    
    if not animate:
        print(cost)
        dur = -1 if not PART_6_TESTING else end_time-start_time
        return path_found, dur, cost

    final_path = []
    for i in range(path.shape[1]-1):
        pose1 = path[:, i][:, np.newaxis]
        pose2 = path[:, i+1][:, np.newaxis]
        if args.robot == 'freeBody':
            new = utils.interpolate_freeBody(pose1, pose2, nearest_neighbors.dist_freeBody)
        else:
            new = utils.interpolate_arm(pose1, pose2, nearest_neighbors.dist_arm)
        final_path = final_path + new



    if args.robot == 'freeBody': 
        fig, axs = plt.subplots(1, 2, figsize=(15, 8))
        axs = axs.flatten()
        for ax in axs:
            ax.set_aspect('equal')
            ax.set_xlim((-10,10))
            ax.set_ylim((-10,10))
            ax.grid()
        component_1.visualize_scene_diff_axes(axs[0], obstacles)
        component_1.visualize_scene_diff_axes(axs[1], obstacles)
        utils.draw_graph(axs[0], graph, args.robot)
        utils.draw_path(axs[1], path, args.robot)

        def drawFrame(pose):
            return utils.draw_rectangle_and_frame(axs[1], pose)
        anim = animation.FuncAnimation(fig, drawFrame, frames=final_path, blit=True, interval=300/len(path))
    else:
        fig, axs = plt.subplots(1, 2, figsize=(15, 8))
        axs = axs.flatten()
        for ax in axs:
            ax.set_aspect('equal')
            ax.grid()        
        axs[0].set_xlim((-np.pi, np.pi))
        axs[0].set_xlim((-np.pi, np.pi))
        axs[1].set_xlim((-10,10))
        axs[1].set_ylim((-10,10))

        utils.draw_graph(axs[0], graph, args.robot)
        component_1.visualize_scene_diff_axes(axs[1], obstacles)
        def drawFrame(pose):
            return utils.draw_2arm_and_frame(axs[1], pose)
        anim = animation.FuncAnimation(fig, drawFrame, frames=final_path, blit=True, interval=300/len(path))
    
    # if EXPORT:
    #     anim.save(f"prm_{args.robot}_{args.map}_path.gif")
    plt.show()
def custum_args_arm(start, goal, mapNum):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['arm', 'freeBody'])
    parser.add_argument('--start', nargs='*', type=float)
    parser.add_argument('--goal', nargs='*', type=float)
    parser.add_argument('--map')
    args = parser.parse_args(["--robot", "arm", "--start", str(start[0]), str(start[1]), "--goal", str(goal[0]), str(goal[1]), "--map", f"map{mapNum}"])
    return args
def custum_args_freeBody(start, goal, mapNum):
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['arm', 'freeBody'])
    parser.add_argument('--start', nargs='*', type=float)
    parser.add_argument('--goal', nargs='*', type=float)
    parser.add_argument('--map')
    # args = parser.parse_args(["--robot", "arm", "--start", "0", "0", "--goal", "3.93", "-3.14", "--map", "map4"])
    args = parser.parse_args(["--robot", "freeBody", "--start", str(start[0]), str(start[1]), str(start[2]), "--goal", str(goal[0]), str(goal[1]), str(goal[2]), "--map", f"map{mapNum}"])
    return args
if __name__ == "__main__":
    if not PART_6_TESTING:
        if EXPORT:
            arm_1_export = custum_args_arm((3.14, 0), (-2, 10.424), 4)
            arm_2_export = custum_args_arm((3.14/8, 0), (-3.14/4*3, 0), 3)
            free_1_export = custum_args_freeBody((9, 9, -4), (9, -9.3, 2), 4)
            free_2_export = custum_args_freeBody((6, 8.5, -10), (-9.2, -2.5, 0), 3)

            main(arm_1_export,  animate=True)
            main(arm_2_export, animate=True)
            main(free_1_export,  animate=True)
            main(free_2_export,  animate=True)
        else:
            a = get_args()
            main(a, animate=True)
    # arm_1_export = custum_args_arm((3.14, 0), (-2, 10.424), 4)
    # arm_2_export = custum_args_arm((3.14/8, 0), (-3.14/4*3, 0), 3)
    # free_1_export = custum_args_freeBody((9, 9, -4), (9, -9.3, 2), 4)
    # free_2_export = custum_args_freeBody((6, 8.5, -10), (-9.2, -2.5, 0), 3)                     
    # main(arm_2_export, animate=True)
    if PART_6_TESTING:
        args_arm = [custum_args_arm((-3.14, 0), (-3.14/4, -3.14/4), 0),
                    custum_args_arm((3.14/4, 0), (-3.14/4*3, 0), 1),
                    custum_args_arm((3.14/4*3, 0), (-3.14/2, 0), 2),
                    custum_args_arm((3.14/8, 0), (-3.14/4*3, 0), 3),
                    custum_args_arm((0, 0), (-2, 10.424), 4)]
        stats_arm = []
        for arg in args_arm:
            data = []
            for _ in range(ITER_TESTING):
                run = main(arg)
                data.append(run)
            data = np.asarray(data)
            avg = np.average(data, axis=0)
            
            a = avg[-1]
            avg[-1] = np.average(data[np.isfinite(data[:, -1])], axis=0)[-1]
            print(a, "vs", avg[-1])
            stats_arm.append(avg)

        args_free = [custum_args_freeBody((9, 9, -2), (-9, -9, 3.14/2), 0),
                 custum_args_freeBody((-9, 9, 5), (9, -9, 3.14/4), 1),
                 custum_args_freeBody((6.5, 1, 3.14/4*3), (-9, 0, 1.3), 2),
                 custum_args_freeBody((6, 8.5, -10), (-9.2, -2.5, 0), 3),
                 custum_args_freeBody((9, 9, -4), (9, -9.3, 2), 4)]
        stats_free = []
        for arg in args_free:
            data = []
            for _ in range(ITER_TESTING):
                run = main(arg)
                data.append(run)
            data = np.asarray(data)
            avg = np.average(data, axis=0)
            
            a = avg[-1]
            avg[-1] = np.average(data[np.isfinite(data[:, -1])], axis=0)[-1]
            print(a, "vs", avg[-1])
            stats_free.append(avg)

        print("Arm Robot Testing")
        for i, (path_found, run_time, path_cost) in enumerate(stats_arm):
            print(f"- Run {i} done {ITER_TESTING} Times")
            print(f"\tPath Found %: {path_found}")
            print(f"\tAVG Run Time (s): {run_time}")
            print(f"\tAVG Path Cost: {path_cost}")
        print("FreeBody Robot Testing")
        for i, (path_found, run_time, path_cost) in enumerate(stats_free):
            print(f"- Run {i} done {ITER_TESTING} Times")
            print(f"\tPath Found %: {path_found}")
            print(f"\tAVG Run Time (s): {run_time}")
            print(f"\tAVG Path Cost: {path_cost}")
