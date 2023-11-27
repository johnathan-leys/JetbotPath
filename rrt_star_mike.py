#!/usr/bin/python3

import numpy as np
import random
import matplotlib.pyplot as plt

class RRT:
    def __init__(self, start, goal, obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles

        self.iterations = 1000
        self.stepsize = 40

        self.radius = 0.05 # bot is a point for now
        self.max_bound = 1.2
        self.min_bound = -0.3

        self.graph = {start : []}
        self.points = []


    def plot_obstacles(self, ax):
        ax.plot(self.start[0], self.start[1], 'r+', \
                markersize = 10, label='start')
        ax.plot(self.goal[0], self.goal[1], 'g*', \
                markersize = 10, label='start')

        for ob in self.obstacles:
            ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], color='black'))
        for pt in self.points:
            ax.plot(pt[0], pt[1], pt[2], markersize=5, label='point')


    def plot_path(self, ax, path):
        for i in range(1,len(path)):
            ax.plot((path[i-1][0], path[i][0]), (path[i-1][1], path[i][1]),
                    'green', linewidth=5)


    def plan(self):
        for i in range(self.iterations):
            rand_pt = np.random.rand(2) * (self.max_bound - self.min_bound) + self.min_bound

            nearest_pt, dist, relatives = self.closest_point_on_graph(self.graph, rand_pt)
            #print(relatives)

            p1 = nearest_pt
            p2 = rand_pt
            run = p2[0] - p1[0]
            rise = p2[1] - p1[1]

            steps = int(np.floor(self.stepsize * dist))
            if steps <= 0: # skip points that are too close
                continue

            all_pts = np.array([(p1[0]+(run*i)/steps , p1[1]+(rise*i)/steps)
                    for i in range(steps+1)])

            collisions = self.points_will_collide(all_pts)
            if collisions[0]: # first point already collides
                continue

            indices, = np.nonzero(collisions)
            if len(indices):
                first_non_colliding = all_pts[indices[0] - 1]
            else:
                first_non_colliding = rand_pt


            # if nearest point is one of the vertices
            if any([np.array_equal(nearest_pt, r) for r in relatives]):
                self.graph.setdefault(tuple(nearest_pt.tolist()), [])\
                    .append(tuple(first_non_colliding.tolist()))
            else:   # there is a break, find the parents
                if any([np.array_equal(relatives[1], r) for r in
                        self.graph[tuple(relatives[0].tolist())]]):
                    # set nearest a child of start
                    self.graph.setdefault(tuple(relatives[0].tolist()), [])\
                        .append(tuple(nearest_pt.tolist()))
                    # get end out of start children
                    self.graph[tuple(relatives[0].tolist())].remove(tuple(relatives[1].tolist()))
                    # set end a child of nearest
                    self.graph.setdefault(tuple(nearest_pt.tolist()), [])\
                        .append(tuple(relatives[1].tolist()))
                    self.graph.setdefault(tuple(nearest_pt.tolist()), [])\
                        .append(tuple(first_non_colliding.tolist()))
                # start is child of end
                elif any([np.array_equal(relatives[0], r) for r in
                        self.graph[tuple(relatives[1].tolist())]]):
                    # set nearest a child of end
                    self.graph.setdefault(tuple(relatives[1].tolist()), [])\
                        .append(tuple(nearest_pt.tolist()))
                    # get start out of end children
                    self.graph[tuple(relatives[1].tolist())].remove(tuple(relatives[0].tolist()))
                    # set start a child of nearest
                    self.graph.setdefault(tuple(nearest_pt.tolist()), [])\
                        .append(tuple(relatives[0].tolist()))
                    self.graph.setdefault(tuple(nearest_pt.tolist()), [])\
                        .append(tuple(first_non_colliding.tolist()))
                # end is child of start
                else:
                    print("oops, please be unreachable :(")

            if self.close_enough_to_goal(first_non_colliding):
                print("iterations reqired: " + str(i))
                path = self.get_final_path(first_non_colliding)
                break

            if i % 50 == 0:
                fig, ax = plt.subplots()
                self.plot_obstacles(ax)
                #self.plot_path(ax, path)
                self.plot_graph(ax, self.graph)
                plt.show()

        fig, ax = plt.subplots()
        self.plot_obstacles(ax)
        self.plot_path(ax, path)
        self.plot_graph(ax, self.graph)
        plt.show()
        return path


    def it_will_collide(self, pt):
        for ob in self.obstacles:
            # find distance between point and obstacle center, compare to
            # obstacle radius and robot radius
            if np.sqrt((pt[0] - ob[0])**2 + (pt[1] - ob[1])**2) < ob[2] + self.radius:
                return True
        return False


    def points_will_collide(self, pts):
        collisions = list(pts)
        for i, pt in enumerate(collisions):
            if self.it_will_collide(pt):
                collisions[i] = True
            else:
                collisions[i] = False

        return collisions


    def close_enough_to_goal(self, pt):
        return np.sqrt((pt[0] - self.goal[0])**2 + \
                   (pt[1] - self.goal[1])**2) < self.goal[2] + 0.07


    def get_final_path(self, pt):
        final_path = [pt]
        # follow parents up
        while (not np.array_equal(final_path[-1], np.array(self.start))):
            for parent, children in self.graph.items():
                for child in children:
                    if np.array_equal(final_path[-1], child):
                        final_path.append(np.array(parent))

        return final_path


    def gen_random_graph(self, nvertices=10, edge_density=0.3):
        graph = {}
        vertices = (np.random.rand(nvertices, 2) * self.max_bound - \
                    self.min_bound) + self.min_bound

        for i, v in enumerate(vertices):
            ind_count = int(np.floor(np.random.randint(1, nvertices) * edge_density))
            ind_list = [np.random.randint(0, nvertices)
                        for n in range(ind_count)]
            if i in ind_list:
                ind_list.remove(i)
                ind_count -= 1

            graph[tuple(v)] = vertices[ind_list]

        return graph

    def plot_graph(self, ax, graph):
        # graph each node, and connect it to its children.
        for node, children in graph.items():
            ax.plot(node[0], node[1], 'y*', markersize=3, label='point')
            for child in children:
                ax.plot((node[0], child[0]), (node[1], child[1]), 'k-')
                ax.plot(child[0], child[1], 'y*', markersize=5, label='point')

    # similar to plot graph but I used this one for testing
    def print_random_testing_graph(self):
        graph = self.gen_random_graph()
        fig, ax = plt.subplots()
        #self.plot_obstacles(ax)

        #for node, children in graph.items():
        #    print("node")
        #    print(node)
        #    print("children")
        #    print(children)

        # graph each node, and connect it to its children.
        for node, children in graph.items():
            ax.plot(node[0], node[1], 'r*', markersize=5, label='point')
            for child in children:
                ax.plot((node[0], child[0]), (node[1], child[1]), 'k-')

        # visualize the closest_point_on_graph func
        test_point = (0.1, 0.5)
        close = self.closest_point_on_graph(graph, test_point)
        #print(close)
        ax.plot(test_point[0], test_point[1], 'y*', markersize=10, label='point')
        ax.plot(close[0][0], close[0][1], 'b*', markersize=10, label='point')
        ax.plot(close[2][0][0], close[2][0][1], 'g*', markersize=6, label='point')
        ax.plot(close[2][1][0], close[2][1][1], 'g*', markersize=6, label='point')

        # visualize the points_will_collide func
        # make a line out of the first graph/child pair it finds, and color
        # based on whether it will intersect an obstacle
        #steps = 10
        #for node in graph.keys():
        #    if len(graph[node]):
        #        p1 = node
        #        p2 = graph[node][0] #first child

        #        run = p2[0] - p1[0]
        #        rise = p2[1] - p1[1]

        #        pts = np.array([(p1[0]+(run*i)/steps , p1[1]+(rise*i)/steps)
        #                       for i in range(steps+1)])

        #        for pt in pts:
        #            ax.plot(pt[0], pt[1], 'y*', markersize=5, label='point')
        #        collisions = self.points_will_collide(pts)
        #        if any(collisions):
        #            ax.plot((pts[0][0], pts[-1][0]), (pts[0][1], pts[-1][1]), 'r-')
        #        else:
        #            ax.plot((pts[0][0], pts[-1][0]), (pts[0][1], pts[-1][1]), 'g-')
        #        break


    def nearest_k_points(self, point, graph, k=2):
        vertices = list(graph.keys())
        vertices_np = np.array(vertices)

        closest = [(np.sqrt((vertex[0] - point[0])**2 + \
                           (vertex[1] - point[1])**2)) \
                    for vertex in vertices]

        closest_indices = np.argpartition(closest, 2)[:2]
        closest = vertices_np[closest_indices]

        return closest

    # taken from RRT implementation ipynb
    # modified to also return the ends of the line segment it's found from
    def closest_point_on_line_segs(self, edges, pt):
        assert edges.shape[-2] == 2
        *N, _, D = edges.shape
        vs, ve = edges[:, 0, :], edges[:, 1, :]
        edge_vec = ve - vs # *N x D
        edge_mag = np.linalg.norm(edge_vec, axis=-1, keepdims=True) #  *N 
        edge_unit = edge_vec / edge_mag # *N x D

        # pt on edge = x = vs + t * edge_unit
        # (x - pt) @ edge_unit = 0
        # ((vs + t * edge_unit) - pt) @
        # edge_unit = 0
        # t = (pt - vs) @ edge_unit
        t = ((pt - vs) * edge_unit).sum(axis=-1, keepdims=True)
        # N x 1
        x = vs + t * edge_unit # *N x D
        dist_e = np.linalg.norm(pt - x, axis=-1)
        dist_vs = np.linalg.norm(vs - pt, axis=-1)
        dist_ve = np.linalg.norm(ve - pt, axis=-1)
        is_pt_on_edge = ((0 <= t) & (t <= edge_mag))[..., 0]
        dist_v = np.minimum(dist_vs, dist_ve)
        dist = np.where( is_pt_on_edge, dist_e, dist_v)
        min_idx = np.argmin(dist)
        closest_point = (x[min_idx]
                        if is_pt_on_edge[min_idx] else vs[min_idx]
                        if (dist_vs[min_idx] < dist_ve[min_idx]) else ve[min_idx])
        return closest_point, dist[min_idx], [vs[min_idx], ve[min_idx]]

    # taken from RRT implementation ipynb
    def closest_point_on_graph(self, G_adjacency_list, pt):
        vertices = list(G_adjacency_list.keys())
        edge_list = sum([[(v, n)
                        for n in nbrs]
                        for v, nbrs in G_adjacency_list.items()], [])

        if len(edge_list):
            return self.closest_point_on_line_segs(np.array(edge_list), pt)
        else:
            verticesnp = np.array(vertices)
            dists_v = np.linalg.norm(verticesnp - pt, axis=-1)
            min_idx = np.argmin(dists_v)
            closest_point_v = verticesnp[min_idx]
            # adding a third output could break other stuff?
            return closest_point_v, dists_v[min_idx],\
                (np.array([0.0, 0.0]), np.array([0.0, 0.0]))

def main():
    # points of interest
    goal = (0.5, .75, 0.02)
    start = (0.0, 0.0)
    # 3rd dimension is radius, assumed to be a circle
    obstacles = [(0.3, 0.5, 0.1), (0.4, 0.2, 0.13), \
                 (0.0, 0.6, 0.15), (0.8, -0.2, 0.1), \
                 (0.9, 0.8, 0.08)]

    rrt = RRT(start, goal, obstacles)
    #rrt.gen_random_graph()
    #rrt.print_random_testing_graph()
    #plt.show()
    path = rrt.plan()
    for item in path:
        print(tuple(item))


if __name__ == '__main__':
    main()
