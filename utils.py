import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from numpy import sin, cos, pi
from typing import Tuple, List
from collections import defaultdict
import heapq
import pprint
from collections import Counter


def angle_to_rotation_matrix(theta: float) -> np.ndarray:
    return np.array([[cos(theta), -sin(theta)], 
                       [sin(theta),  cos(theta)]])
def xy_to_t(x, y) -> np.ndarray:
    # col vector
    return np.array([[x, y]]).T
def t_to_xy(t: np.ndarray) -> np.ndarray:
    # row vector, which will be unpacked to x,y
    return t.T[0]
def euler_to_transformation_matrix(pose: np.ndarray) -> np.ndarray:
    # p = [x, y, theta].T col vector 
    R = angle_to_rotation_matrix(pose[2, 0])
    t = xy_to_t(pose[0, 0], pose[1, 0])
    top = np.concatenate((R, t), axis=1)
    bot = np.eye(1, 3, 3-1)
    return np.concatenate((top, bot), axis=0)
def rect_to_points(w, h, x, y, theta) -> np.ndarray:
    T = euler_to_transformation_matrix(np.array([[x, y, theta]]).T)
    top = np.array(([-w/2, -w/2, w/2, w/2],
                    [-h/2, h/2, h/2, -h/2]))
    bot = np.ones(4)
    points = np.vstack((top, bot))
    # returns 2x4 matrix. 4 vertices, and x,y per vertex
    return (T@points)[0:2,:]
def freeBody_to_points(pose) -> np.ndarray:
    return rect_to_points(.5, .2, *pose.flatten())
def arm_to_points(pose) -> Tuple[np.ndarray, np.ndarray]:
    w, l1, l2 = .3, 2, 1.5
    top = np.array(([0, 0, l1, l1],
                    [-w/2, w/2, w/2, -w/2]))
    bot = np.ones(4)
    points_l1 = np.vstack((top, bot))
    top = np.array(([0-w/2, 0-w/2, l2-w/2, l2-w/2], 
                    [-w/2, w/2, w/2, -w/2]))
    bot = np.ones(4)
    points_l2 = np.vstack((top, bot))
    # print(pose)
    # print(np.array([[0, 0, pose[0,0]]]))
    T1 = euler_to_transformation_matrix(np.array([[0, 0, pose[0,0]]]).T)
    T1_2 =  euler_to_transformation_matrix(np.array([[l1, 0, pose[1,0]]]).T)
    T2 = T1 @ T1_2
    
    return (T1@points_l1)[0:2], (T2@points_l2)[0:2]
def random_euler_angle() -> float:
    return np.random.uniform(-pi, pi)
def random_pose_xytheta() -> np.ndarray:
    x = np.random.uniform(-10, 10)
    y = np.random.uniform(-10, 10)
    theta = random_euler_angle()
    # col vector
    return np.array([[x, y, theta]]).T
def random_pose_thetas() -> np.ndarray:
    theta1 = random_euler_angle()
    theta2 = random_euler_angle()
    # col vector
    return np.array([[theta1, theta2]]).T
def random_wh() -> np.ndarray:
    return np.random.uniform(.5, 2, size=2)
def normalize_angle(angle: float | np.ndarray):
    if isinstance(angle, bytes):
        angle = float(angle)
    return (angle + pi) % (2*pi) - pi
def draw_rectangle_and_frame(ax, pose: np.ndarray, w=.5, h=.2, ec='r'):
    theta_deg = np.rad2deg(pose[2])
    r_patch = ax.add_patch(patches.Rectangle((pose[0, 0]-w/2, pose[1, 0]-h/2), w, h, angle=theta_deg, rotation_point=(pose[0, 0], pose[1, 0]), ec=ec, fc='none'))

    q_patch = ax.quiver(pose[0], pose[1], np.cos(pose[2]), np.sin(pose[2]), width=.003, color=ec)
    return r_patch, q_patch
def draw_2arm_and_frame(ax, pose: np.ndarray, l1=2, l2=1.5, w=.3, ec='r'):
    thetas_deg = np.rad2deg(pose)
    artists = []
    artists.append(ax.quiver(0, 0, 1, 0, width=.003, color='b') )
    artists.append(ax.add_patch(patches.Rectangle((0, -w/2), l1, w, angle=thetas_deg[0, 0], rotation_point=(0, 0), ec=ec, fc='none')))
    artists.append(ax.quiver(0, 0, np.cos(pose[0, 0]), np.sin(pose[0, 0]), width=.003, color=ec))
    
    R1 = angle_to_rotation_matrix(pose[0, 0]) 
    t1 = xy_to_t(l1, 0)
    l1_x_end, l1_y_end = t_to_xy(R1@t1)
    artists.append(ax.quiver(l1_x_end, l1_y_end, np.cos(pose[0, 0]), np.sin(pose[0, 0]), width=.003, color=ec))
    artists.append(ax.add_patch(patches.Rectangle((l1_x_end-w/2, l1_y_end-w/2), l2, w, angle=thetas_deg[0, 0]+thetas_deg[1, 0], rotation_point=(l1_x_end, l1_y_end), ec=ec, fc='none')))
    artists.append(ax.quiver(l1_x_end, l1_y_end, np.cos(pose[0, 0] + pose[1, 0]), np.sin(pose[0, 0] + pose[1, 0]), width=.003, color=ec))
    R2 = angle_to_rotation_matrix(pose[1, 0])
    t2 = xy_to_t(l2-w/2, 0)
    end_x, end_y = t_to_xy(R1@R2@t2 + R1@t1)
    artists.append(*ax.plot(end_x, end_y, '%so'%ec, ms=1),)
    # p1, p2 = arm_to_points(pose)
    # ax.add_patch(patches.Polygon(p1.T, fill=False))
    # ax.add_patch(patches.Polygon(p2.T, fill=False))
    return artists
def draw_path(ax, path, robot_type):
    if robot_type == 'freeBody':
        _draw_path_freeBody(ax, path)
    else:
        _draw_path_arm(ax, path)
def _draw_path_freeBody(ax, path):
    # path is DoFxNPoints
    x_vertices, y_vertices = path[0], path[1]
    ax.plot(x_vertices, y_vertices, color='red', linestyle='solid', marker='o', markerfacecolor='blue')
    pass
def _draw_path_arm(ax, path):
    pass
def draw_graph(ax, graph, robot_type):
    if robot_type == 'freeBody':
        _draw_graph_freeBody(ax, graph)
    else:
        _draw_graph_arm(ax, graph)
def _draw_graph_freeBody(ax, graph):
    edges = graph.get_edges()

    for e in edges:
        ax.plot(e[:, 0], e[:, 1], c='r')
    
    vertices = graph.get_vertices()  #100x3
    x_vertices, y_vertices = vertices[:, 0], vertices[:, 1]
    ax.scatter(x_vertices, y_vertices)
    pass


def _draw_graph_arm(ax, graph):   
    # dists = []
    for edge in graph.get_edges():
        d = np.linalg.norm(edge[0]-edge[1])
        # dists.append(d)
        # dists.append(d)
        if d <= 1:
            ax.plot(edge[:, 0], edge[:, 1], marker='o', c='r', mfc='b')
        else:
            pass
            # prof said we dont have to plot these types of edges 
    # for d in sorted(dists):
    #     print(d)
    # frequency = Counter([int(i) for i in dists])
    # # Printing the frequency
    # for number, count in frequency.items():
    #     print(f"{number}: {count} times")
def in_rad(currr, goal, goal_rad, dist):
    if goal_rad == 0:
        return currr == goal
    else:
        return dist(np.array(currr), np.array(goal)) <= goal_rad 
def A_star(graph, dist, _start, _goal, goal_rad = 0.):
    # dist func takes in 2 poses. each pose is a row vector
    # returns a List[np.ndarray row vectors]
    start = tuple(x for x in _start.flatten())
    goal = tuple(x for x in _goal.flatten())
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_scores = defaultdict(lambda: float('inf'))
    g_scores[start] = 0
    f_scores = defaultdict(lambda: float('inf'))
    f_scores[start] = dist(np.array(start), np.array(goal))
    closed_set = set()
    while open_set:
        current = heapq.heappop(open_set)[1]
        closed_set.add(current)

        if in_rad(current, goal, goal_rad, dist):
        # if current == goal:
            print("Path FOund now calculation")
            path = []
            c= current
            while c in came_from:
                path.append(np.array(c))
                c = came_from[c]
            path.append(np.array(start))
            path.reverse()

            return path, g_scores[current]

        for neighbor in graph._graph[current]:
            if neighbor in closed_set:
                continue
            tentative_g_score = g_scores[current] + dist(np.array(current), np.array(neighbor))

            if tentative_g_score < g_scores[neighbor]:
                came_from[neighbor] = current
                g_scores[neighbor] = tentative_g_score
                f_scores[neighbor] = g_scores[neighbor] +  dist(np.array(neighbor), np.array(goal))
                heapq.heappush(open_set, (f_scores[neighbor], neighbor))

    return None, float('inf')

def steer_interpolate_freeBody(start_pose: np.ndarray, goal_pose: np.ndarray, dist_func, d) -> List[np.ndarray]:
    # returns a (path, pose_new). Path is a path to a pose_new that is on the straight line from start_pose to goal_pose, d distance away
    # print(start_pose)
    # print(goal_pose)
    epslon_dist = min(d/3, .1) #guarentes that the smallest path will be discritized into atleast 3 frames
    dst = dist_func(start_pose.flatten(), goal_pose.flatten())
    frames = max(int(dst / epslon_dist), 1)
   
    # frames = 30
    start_pose[2, 0] = normalize_angle(start_pose[2, 0])
    goal_pose[2, 0] = normalize_angle(goal_pose[2, 0])
    path = [start_pose]
    delta_pose = (goal_pose-start_pose)
    delta_pose[2, 0] = normalize_angle(delta_pose[2, 0])

    delta_per_frame = delta_pose/frames
    # print(delta_per_frame)
    # print("frames: ", frames)
    delta_per_frame_dist = dist_func(np.zeros((1,3)).flatten(), delta_per_frame.flatten())
    # print(d, "/", delta_per_frame_dist, "==")
    # print( "->", int(d / delta_per_frame_dist))
    frames_needed = max(int(d / delta_per_frame_dist), 1)
    moving = min(frames, frames_needed)
    # print("moving: %d | (cal, need): %s", min(frames, frames_needed), (frames, frames_needed))
    for f in range(moving):
        next_pose = path[f] + delta_per_frame
        next_pose[2, 0] = normalize_angle(next_pose[2, 0])
        path.append(next_pose)
    return path, path[-1]
def steer_interpolate_arm(start_pose: np.ndarray, goal_pose: np.ndarray, dist_func, d) -> List[np.ndarray]:
    # print(start_pose, goal_pose)
    epslon_dist = min(d/3, .1) #guarentes that the smallest path will be discritized into atleast 3 frames
    dst = dist_func(start_pose.flatten(), goal_pose.flatten())
    frames = max(int(dst / epslon_dist), 1)
    # frames = 30
    start_pose = normalize_angle(start_pose)
    goal_pose = normalize_angle(goal_pose)
    path = [start_pose]
    delta_pose = normalize_angle(goal_pose-start_pose) 
    delta_per_frame = delta_pose/frames
    delta_per_frame_dist = dist_func(np.zeros((1,2)).flatten(), delta_per_frame.flatten())
    if delta_per_frame_dist != 0:
        frames_needed = max(int(d / delta_per_frame_dist), 1)
        moving = min(frames, frames_needed)
    else: 
        moving = frames
    for f in range(moving):
        next_pose = path[f] + delta_per_frame
        path.append(next_pose)
    return path, path[-1]
def interpolate_freeBody(start_pose: np.ndarray, goal_pose: np.ndarray, dist) -> List[np.ndarray]:
    epslon_dist = .1
    dst = dist(start_pose.flatten(), goal_pose.flatten())
    frames = max(int(dst / epslon_dist), 1)
   
    # frames = 30
    start_pose[2, 0] = normalize_angle(start_pose[2, 0])
    goal_pose[2, 0] = normalize_angle(goal_pose[2, 0])
    path = [start_pose]
    delta_pose = (goal_pose-start_pose)
    delta_pose[2, 0] = normalize_angle(delta_pose[2, 0])

    delta_per_frame = delta_pose/frames
    for f in range(frames):
        next_pose = path[f] + delta_per_frame
        next_pose[2, 0] = normalize_angle(next_pose[2, 0])
        path.append(next_pose)
    return path
def interpolate_arm(start_pose: np.ndarray, goal_pose: np.ndarray, dist) -> List[np.ndarray]:
    epslon_dist = .1
    dst = dist(start_pose.flatten(), goal_pose.flatten())
    frames = max(int(dst / epslon_dist), 1)
    # frames = 30
    start_pose = normalize_angle(start_pose)
    goal_pose = normalize_angle(goal_pose)
    path = [start_pose]
    delta_pose = normalize_angle(goal_pose-start_pose) 
    delta_per_frame = delta_pose/frames
    for f in range(frames):
        next_pose = path[f] + delta_per_frame
        path.append(next_pose)
    return path 
# def visualize_path(fig, ax, path: List[np.ndarray], obsctables, draw_robot, vis_scene, export = False, fileName = ""):
#     # Animation Code START
#     def draw(pose):
#         print(pose.shape)
#         ax.cla()
#         ax.set_xlim((-10,10))
#         ax.set_ylim((-10,10))
#         plt.grid()
#         draw_robot(ax, pose)
#         vis_scene(ax, obsctables)
#     anim =  animation.FuncAnimation(fig=fig, func=draw, frames=path, interval=3000/len(path))
#     if export:
#         anim.save(fileName)
#     return anim
class Graph():
    def __init__(self, directed=False, animation=False):
        self._directed = directed
        self._animated = animation
        self._graph = defaultdict(set)
        self._ordered_list = []
    def add_edge(self, pose1, pose2):
        self.add_vertex(pose1)
        self.add_vertex(pose2)
        pose1 = tuple(x for x in pose1.flatten())
        pose2 = tuple(x for x in pose2.flatten())
        self._graph[pose1].add(pose2)
        if not self._directed:
            self._graph[pose2].add(pose1)

        if self._animated:
            self._ordered_list.append((pose1, pose2))

    def add_vertex(self, pose):
        pose = tuple(x for x in pose.flatten())
        if pose not in self._graph:
            self._graph[pose] = set()
    def get_vertices(self):
        # returns a N by X matrix. X = #of DoF, N = number of vertices
        return np.array(list(self._graph.keys()))
    def get_edges(self):
        # returns a 3d array of NEdges x 2(pairs) x DoF
        edges = []
        for u in self._graph:
            for v in self._graph[u]:
                edges.append((u, v))
        return np.array(edges)
    def get_insertion_order_edges(self) -> np.ndarray:
        print("Len inside", len(self._ordered_list))
        return np.array(self._ordered_list)
    def in_graph(self, test_pose):
        test_pose = tuple(x for x in test_pose.flatten())
        return test_pose in self._graph
    def __str__(self):
        return str(self._graph)