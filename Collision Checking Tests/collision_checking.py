import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import argparse
import utils
import component_1
import matplotlib.animation as animation
NO_COL = 'g'
COL = 'r'
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['arm', 'freeBody'])
    parser.add_argument('--map')
    # args = parser.parse_args(["--robot", "freeBody", "--map", "map4"])
    args = parser.parse_args()
    return args
def SAT(robot_vertices, o_vertices):
    for polygon in [robot_vertices, o_vertices]:
        edges = polygon-np.roll(polygon, 1, axis=1)
        len_per_edge = np.linalg.norm(edges, axis=0)
        norm_edges = edges/len_per_edge
        perpendicular_axises = np.vstack((-norm_edges[1], norm_edges[0]))
        # for e in perpendicular_axises.T:
        #     ax.axline((0, 0), (e[0], e[1]), c='g')
        for axis in perpendicular_axises.T:
            # ax.axline((0, 0), (axis[0], axis[1]))
            # project each poly onto axis which is a row vector of norm 1
            projected_robot = axis @ robot_vertices
            projected_obs = axis @ o_vertices
            # check if overlap
            min_robot, max_robot = min(projected_robot), max(projected_robot)
            min_obs, max_obs = min(projected_obs), max(projected_obs)
            if max_robot < min_obs or max_obs < min_robot:
                # sep axis is found
                return True
    return False
def check_col_freeBody(pose, obstacles):
    # (True, [colider]) if col
    # (False, []) if no col
    coliders = []
    robot_vertices = utils.freeBody_to_points(pose)  
    for o in obstacles:
        o_vertices = utils.rect_to_points(*o)
        sep_axis_found =  SAT(robot_vertices, o_vertices)
        if not sep_axis_found:
            coliders.append(o)
    num_of_colider = len(coliders)
    if num_of_colider == 0:
        return False, coliders
    else:
        return True, coliders

def check_col_arm(pose, obstacles):
    # (True, [colider]) if col
    # (False, []) if no col
    coliders = []
    link1_vertices, link2_vertices = utils.arm_to_points(pose)
    for o in obstacles:
        o_vertices = utils.rect_to_points(*o)
        sep_axis_found_l1 =  SAT(link1_vertices, o_vertices)
        sep_axis_found_l2 =  SAT(link2_vertices, o_vertices)
        if not (sep_axis_found_l1 and sep_axis_found_l2):
            coliders.append(o)

    num_of_colider = len(coliders)
    if num_of_colider == 0:
        return False, coliders
    else:
        return True, coliders
def test():
    polygon = np.ndarray([[]])
def animationeee():
    args = get_args()
    obstacles = component_1.scene_from_file(args.map)
    fig, ax = plt.subplots(1, 1, figsize=(15, 8))
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    ax.grid()   

    def draw(i):
        ax.cla()
        ax.set_xlim((-10,10))
        ax.set_ylim((-10,10))
        ax.grid()
        component_1.visualize_scene_diff_axes(ax, obstacles)
        if args.robot == 'arm':
            pose = utils.random_pose_thetas()
            col, coliders = check_col_arm(pose, obstacles)
            if col:
                component_1.visualize_scene_diff_axes(ax, coliders, ec=COL)
                utils.draw_2arm_and_frame(ax, pose, ec=COL)
            else:
                utils.draw_2arm_and_frame(ax, pose, ec=NO_COL)

        elif args.robot == 'freeBody':
            pose = utils.random_pose_xytheta()
            col, coliders = check_col_freeBody(pose, obstacles)
            if col:
                component_1.visualize_scene_diff_axes(ax, coliders, ec=COL)
                utils.draw_rectangle_and_frame(ax, pose, ec=COL)
            else:
                utils.draw_rectangle_and_frame(ax, pose, ec=NO_COL)
        pass
    anim = animation.FuncAnimation(fig, draw, 10, repeat=False, interval=1000)
    # anim.save(f"col_freeBody_{args.map}.gif")
    plt.show()
    pass
if __name__ == '__main__':
    # test()
    animationeee()

    # args = get_args()
    # obstacles = component_1.scene_from_file(args.map)
    
    # fig, axs = plt.subplots(2, 5, figsize=(15, 8))
    # axs = axs.flatten()
    # for i in range(10):
    #     axs[i].set_xlim(-10, 10)
    #     axs[i].set_ylim(-10, 10)
    #     axs[i].set_title("Check %d" % i)
    #     axs[i].set_aspect('equal')
    #     axs[i].grid()

    #     component_1.visualize_scene_diff_axes(axs[i], obstacles)
    #     if args.robot == 'arm':
    #         pose = utils.random_pose_thetas()
    #         col, coliders = check_col_arm(pose, obstacles)
    #         if col:
    #             component_1.visualize_scene_diff_axes(axs[i], coliders, ec=COL)
    #             utils.draw_2arm_and_frame(axs[i], pose, ec=COL)
    #         else:
    #             utils.draw_2arm_and_frame(axs[i], pose, ec=NO_COL)

    #     elif args.robot == 'freeBody':
    #         pose = utils.random_pose_xytheta()
    #         col, coliders = check_col_freeBody(pose, obstacles)
    #         if col:
    #             component_1.visualize_scene_diff_axes(axs[i], coliders, ec=COL)
    #             utils.draw_rectangle_and_frame(axs[i], pose, ec=COL)
    #         else:
    #             utils.draw_rectangle_and_frame(axs[i], pose, ec=NO_COL)

        
    # plt.show()

