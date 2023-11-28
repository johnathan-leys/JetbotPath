#!/usr/bin/python3

# this program is a WIP RRT* implementation, it's an RRT implementation but with
# costs added to the graph, and some other helper functions currently being
# added as well.

import numpy as np
import random
import matplotlib.pyplot as plt
import pprint

class RRT:
    def __init__(self, start, goal, obstacles):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles

        self.iterations = 1000
        self.occasional_plot = True
        self.stepsize = 50
        self.point_separation = 0.025 #how far apart nodes on line are
        self.neighborhood_radius = 0.25

        self.radius = 0.05 # bot is a point for now
        self.max_bound = 1.2
        self.min_bound = -0.3

        self.graph = {(start, 0.0): []}
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
            ax.plot((path[i-1][0], path[i][0]),
                    (path[i-1][1], path[i][1]), 'green', linewidth=5)



    def plan(self):
        for i in range(self.iterations):
            rand_pt = tuple(np.random.rand(2)*(self.max_bound-self.min_bound)+self.min_bound)

            nearest_pt, cost = self.closest_point_on_graph(self.graph, rand_pt)

            steps = int(np.floor(self.stepsize * cost))
            if steps <= 0: # skip points that are too close
                continue

            all_pts = self.create_point_chain(nearest_pt, rand_pt, steps)
            collisions = self.points_will_collide(all_pts)
            if collisions[0]: # first point already collides
                continue
            indices, = np.nonzero(collisions)

            if len(indices):
                first_non_colliding = all_pts[indices[0] - 1][0]
            else:
                # take cost from initial calc above
                first_non_colliding = rand_pt

            best_pt, neighbors = self.find_neighbors(nearest_pt, cost, first_non_colliding)
            self.add_line_to_graph(first_non_colliding, nearest_pt)

            # we reached the goal, quit the planning function
            if self.close_enough_to_goal(first_non_colliding):
                print("iterations reqired: " + str(i))
                path = self.get_final_path(first_non_colliding)
                break

            if i % 25 == 0 and self.occasional_plot:
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



    # could be better to start with the neighbors of the nearest pt?
    def find_neighbors(self, nearest_pt, start_dist, new_pt):
        # start by assuming the nearest pt is the best
        best_dist = start_dist
        best_pt = nearest_pt
        neighbors = []

        for pt in self.graph.keys():
            dist = self.pythagoras(new_pt, pt[0])
            if dist < self.neighborhood_radius:
                neighbors.append(pt)
                if dist < best_dist:
                    best_pt = pt

        return best_pt, neighbors



    def add_line_to_graph(self, new_pt, nearest_pt):
        # line of points from nearest to new, separated by at least point_sep
        npts = int(self.pythagoras(new_pt, nearest_pt[0])/self.point_separation)+1
        if npts == 1:
            npts = 2

        point_chain = self.create_point_chain(nearest_pt, new_pt, npts)

        # add the point chain to nearest pt's children
        self.graph[nearest_pt].append(point_chain[0])
        for i in range(npts-1):
            self.graph.setdefault(point_chain[i], [point_chain[i+1]])
        self.graph.setdefault(point_chain[npts-1], [])



    def create_point_chain(self, start, end, npts):
        run = end[0] - start[0][0]
        rise = end[1] - start[0][1]
        starting_cost = start[1]
        cost_per_pt = np.sqrt(run**2 + rise**2)

        chain = [((start[0][0] + (i*run)/npts,
                   start[0][1] + (i*rise)/npts),
                   starting_cost + i*cost_per_pt/npts) for i in range(1, npts+1)]

        return chain



    # distance between two points
    def pythagoras(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)



    def it_will_collide(self, pt):
        for ob in self.obstacles:
            # find distance between point and obstacle center, compare to
            # obstacle radius and robot radius
            if np.sqrt((pt[0][0] - ob[0])**2 + (pt[0][1] - ob[1])**2) < ob[2] + self.radius:
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
                    if np.array_equal(final_path[-1], child[0]):
                        final_path.append(np.array(parent[0]))

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
        child_list = []
        weight_list = []
        for node, children in graph.items():
            ax.plot(node[0][0], node[0][1], 'y*', markersize=3, label='point')
            for child in children:
                ax.plot((node[0][0], child[0][0]), (node[0][1], child[0][1]), 'k-')
                #ax.plot(child[0][0], child[0][1], 'y*', markersize=5, label='point')
                child_list.append((child[0][0], child[0][1]))
                weight_list.append(child[1])

        # plot according to weights!
        plt.scatter([x[0] for x in child_list], [x[1] for x in child_list],
                    c=weight_list, cmap='YlOrRd')



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



    # returns closest vertex on graph
    def closest_point_on_graph(self, G, new_pt):
        # set large distance and origin for initial value
        closest = ((0.0, 0.0), 0.0)
        smallest_dist = abs(self.max_bound) + abs(self.min_bound)

        #don't look at the chldren of parents w/o children
        for vertex in G.keys():
            dist = self.pythagoras(new_pt, vertex[0])
            if dist < smallest_dist:
                closest = vertex
                smallest_dist = dist

        # check to see if the key has any children
        for leaf in G[closest]:
            dist = self.pythagoras(new_pt, leaf[0])
            if dist < smallest_dist:
                closest = leaf
                smallest_dist = dist

        return closest, dist



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
    for v in rrt.graph.keys():
        if v[0][0] != v[1]:
            print(v)
    #pp = pprint.PrettyPrinter(compact=True)
    #pp.pprint(rrt.graph)

if __name__ == '__main__':
    main()
