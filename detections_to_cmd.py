import rclpy
import sys
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import numpy as np
import math
import random
import matplotlib.pyplot as plt
import time

twist = Twist()
jetbot_path = []
goal_angle = 0


class RRT:
    def __init__(self, start, goal, obstacles, iterations=1000,
                 neighborhood_radius=0.1, quit_on_goal=True,
                 seeded=False, seed=1234):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles

        self.iterations = iterations
        self.seeded = seeded
        self.seed = seed

        self.occasional_plot = False
        self.quit_on_goal = quit_on_goal
        self.stepsize = 25
        self.point_separation = 0.05 #how far apart nodes on line are
        self.neighborhood_radius = neighborhood_radius

        self.radius = 0.05 # bot is a point for now
        #TODO: add a better bound system
        self.max_bound = 1
        self.min_bound = -1
        self.huge_cost = 1000

        self.graph = {(start, 0.0): []}
        self.points = []
        self.path = []



    # the "main event" of the class, this runs through the RRT* algorithm
    #
    # TODO: collision detection should be moved to a method.
    #
    def plan(self):
        if self.seeded == True:
            np.random.seed(self.seed)

        for i in range(self.iterations):
            rand_pt = tuple(np.random.rand(2)*(self.max_bound-self.min_bound)+self.min_bound)

            nearest_pt, cost = self.closest_point_on_graph(self.graph, rand_pt)

            # chekc for collisions, take the first non-colliding point
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
                first_non_colliding = rand_pt # take cost from initial calc above

            # find the lowest-cost point to connect to
            best_pt, new_pt_cost, neighbors = self.find_neighbors(nearest_pt, first_non_colliding)

            # check collisions again with this new connection
            steps = int(np.floor(self.stepsize * new_pt_cost))
            if steps <= 0: # skip points that are too close
                continue
            all_pts = self.create_point_chain(best_pt, first_non_colliding, steps)
            collisions = self.points_will_collide(all_pts)
            if any(collisions): # first point already collides
                continue

            added_pt = self.add_line_to_graph(first_non_colliding, best_pt)

            # if its more cost effective to reach neighbor thru new_pt, reroute
            for n in neighbors:
                n_cost = new_pt_cost + self.pythagoras(added_pt[0], n[0])
                if n_cost < n[1]:

                    # check for collisions
                    steps = int(np.floor(self.stepsize * n_cost))
                    if steps <= 0: # skip points that are too close
                        continue
                    all_pts = self.create_point_chain(n, added_pt[0], steps)
                    collisions = self.points_will_collide(all_pts)
                    if any(collisions):
                        continue

                    new_n = (n[0], n_cost)
                    self.update_parent(self.graph, n, new_n, added_pt)
                    self.graph[added_pt].append(new_n)


            # we reached the goal, quit the planning function
            if self.close_enough_to_goal(first_non_colliding) and self.quit_on_goal:
                print("iterations reqired: " + str(i))
                self.path = self.get_final_path(first_non_colliding)

                fig, ax = plt.subplots()
                ax.set_title("i = " + str(i) + "(quit on goal found)")
                self.plot_obstacles(ax)
                self.plot_path(ax, self.path)
                self.plot_graph(ax, self.graph)

                return self.path

            if i % 1 == 0 and self.occasional_plot:
                fig, ax = plt.subplots()
                self.plot_obstacles(ax)
                #self.plot_path(ax, path)
                self.plot_graph(ax, self.graph)
                plt.show()

        # find the closest point to the goal and use that for the path
        best_path_end = [0, 0]
        best_cost = self.huge_cost
        for parent, child in self.graph.items():
            for c in child:
                if self.close_enough_to_goal(c[0]):
                    if c[1] < best_cost:
                        best_cost = c[1]
                        best_path_end = list(c[0])


        if best_cost < self.huge_cost:
            print("iterations completed: " + str(self.iterations))
            self.path = self.get_final_path(tuple(best_path_end))

            fig, ax = plt.subplots()
            ax.set_title("i = " + str(self.iterations))
            self.plot_obstacles(ax)
            self.plot_path(ax, self.path)
            self.plot_graph(ax, self.graph)
            return self.path

        else:
            print("unable to find path in " + str(self.iterations) + " iterations");

            return (0.0, 0.0)




    # this will remove the old pt from the old parent, and the new pt with the
    # new cost will be assigned to new_parent, and the new costs will be
    # propagated to its children
    #
    # new parent should already exist in the graph.
    #
    # very slow, would be much better with a Node class that had a pointer to
    # its parent. instead here it searches the entire graph for the point of
    # interest, removes it from it's parent's list and add it to new parent's
    # list.
    #
    # also extra slow because the graph is searched a bunch of times in order to
    # update the children's costs. I just can't be bothered right now to write
    # up a tree structure and its methods ... :(
    #
    #   input ->    dict G with parent child relations of points, with costs
    #               pt ((float x, float y), cost)
    #               new_parent ((float x, float y), cost)
    def update_parent(self, G, old_pt, new_pt, new_parent):
        for parent, children in G.items():
            for c in children:
                # if the point is in the list of children
                if c[0][0] == old_pt[0][0] and c[0][1] == old_pt[0][1]:
                    # remove oldpt from its parent
                    G[parent] = [x for x in G[parent] if (x[0][0] != old_pt[0][0] and
                                       x[0][1] != old_pt[0][1])]

                    G[new_parent].append(new_pt)
                    if new_pt in G.keys():
                        self.update_children_costs(self.graph, new_pt, old_pt[1]-new_pt[1])

                    # each point should only be one child?
                    return True
        return False # hopefully unreachable if both points exist in G?



    # slow cost updating. see description of self.update_parent()
    def update_children_costs(self, G, parent, cost_diff):
        for child in G[parent]:
            # hack
            try:
                G[child]
            except:
                new_cost = child[1] - cost_diff
                child = (child[0], new_cost)

            self.update_children_costs(G, child, cost_diff)

            return
        return



    # returns the "best" point to connect the new point to, based on the
    # previous point's costs, as well as all points that lie in the neighborhood
    # of the new pt
    #
    #   input ->    nearest_pt: ((float x, float y), float cost)
    #               new_pt: (float x, float y)
    #   output ->   best_pt: ((float x, float y), float cost)
    #               best_cost: float, the best cost to get to the new_pt
    #               neighbors: [((float x, float y), float cost), ... ]
    def find_neighbors(self, nearest_pt, new_pt):
        # start by assuming the nearest pt is the best
        best_cost = nearest_pt[1] + self.pythagoras(nearest_pt[0], new_pt)
        best_pt = nearest_pt
        neighbors = []

        for pt in self.graph.keys():
            dist = self.pythagoras(new_pt, pt[0])
            if dist < self.neighborhood_radius:
                neighbors.append(pt)
                cost = pt[1] + self.pythagoras(pt[0], new_pt)
                if cost < best_cost:
                    best_cost = cost
                    best_pt = pt

        return best_pt, best_cost, neighbors



    # adds line of points from nearest to new, separated by at least point_sep,
    # to the graph. the costs of the added point are based on the starting
    # point.
    #
    #   input ->    new_pt: (float x, float y)
    #               nearest_pt: ((float x, float y), float cost)
    def add_line_to_graph(self, new_pt, nearest_pt):
        npts = int(self.pythagoras(new_pt, nearest_pt[0])/self.point_separation)+1
        if npts == 1:
            npts = 2

        point_chain = self.create_point_chain(nearest_pt, new_pt, npts)

        # add the point chain to nearest pt's children
        self.graph[nearest_pt].append(point_chain[0])
        for i in range(npts-1):
            self.graph.setdefault(point_chain[i], [point_chain[i+1]])
        self.graph.setdefault(point_chain[npts-1], [])

        # get the location and cost of new_pt
        return point_chain[npts-1]



    # creates a list of points and associated costs, starting from start and
    # ending at end. this is used for adding points to the graph and also for
    # the discretized collision detection.
    #
    #   input ->    start: tuple (float x, float y)
    #               end: tuple ((float x, float y), float cost)
    #   output ->   chain: list [((float x, float y), float cost), ... ]
    def create_point_chain(self, start, end, npts):
        run = end[0] - start[0][0]
        rise = end[1] - start[0][1]
        starting_cost = start[1]
        cost_per_pt = np.sqrt(run**2 + rise**2)

        chain = [((start[0][0] + (i*run)/npts,
                   start[0][1] + (i*rise)/npts),
                   starting_cost + i*cost_per_pt/npts) for i in range(1, npts+1)]

        return chain



    # calculates distance between two points
    #
    #   input ->    p1: (float x, float y)
    #               p2: (float x, float y)
    #   output ->   float
    def pythagoras(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)



    # returns closest vertex on graph
    #
    #   input ->    G: dictionary, the graph to search for nearest points
    #               new_pt: (float x, float y)
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



    # checks collision of a single point
    #
    #   input ->    pt: ((float x, float y), float cost)
    #   output ->   bool, True for collision occur
    def it_will_collide(self, pt):
        for ob in self.obstacles:
            # find distance between point and obstacle center, compare to
            # obstacle radius and robot radius
            if np.sqrt((pt[0][0] - ob[0])**2 + (pt[0][1] - ob[1])**2) < ob[2] + self.radius:
                return True
        return False



    # check collision of a list of points, returns a list of same length as pts
    # that contains whether or not each point will collide
    #
    #   input ->    pts: [((float x, float y), float cost)]
    #   output ->   collisions [bool, ...] True for collision occur.
    def points_will_collide(self, pts):
        collisions = list(pts)
        for i, pt in enumerate(collisions):
            if self.it_will_collide(pt):
                collisions[i] = True
            else:
                collisions[i] = False

        return collisions



    # check if a point is close enough to the goal
    #
    #   input ->    pt: (float x, float y)
    #   output ->   bool, true for close to goal
    def close_enough_to_goal(self, pt):
        return np.sqrt((pt[0] - self.goal[0])**2 + \
                       (pt[1] - self.goal[1])**2) < self.goal[2] + 0.02



    # traverse the graphs parents in order to get path from pt to start. This is
    # kind of a shitty function and will hang in infinite loop if it doesnt
    # get back to the start
    #
    #   input ->    pt: (float x, float y), must be a vertex in the graph
    def get_final_path(self, pt):
        final_path = [pt]
        # follow parents up
        while (not np.array_equal(final_path[-1], np.array(self.start))):
            for parent, children in self.graph.items():
                for child in children:
                    if np.array_equal(final_path[-1], child[0]):
                        final_path.append(np.array(parent[0]))

        return final_path



    # adds the defined obstacles as well as the start and end points to the
    # axes passed to it.
    def plot_obstacles(self, ax):
        ax.plot(self.start[0], self.start[1], 'r+', \
                markersize = 15, label='start')
        ax.plot(self.goal[0], self.goal[1], 'g*', \
                markersize = 12, label='start')

        for ob in self.obstacles:
            ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], color='black'))
        for pt in self.points:
            ax.plot(pt[0], pt[1], pt[2], markersize=5, label='point')



    # visualizes the final path
    def plot_path(self, ax, path):
        for i in range(1,len(path)):
            ax.plot((path[i-1][0], path[i][0]),
                    (path[i-1][1], path[i][1]), 'green', linewidth=6)



    # testing func
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



    # places graph on axes, and colors based on cost
    def plot_graph(self, ax, graph):
        # graph each node, and connect it to its children.
        child_list = []
        weight_list = []
        for node, children in graph.items():
            ax.plot(node[0][0], node[0][1], 'y*', markersize=3, label='point')
            for child in children:
                ax.plot((node[0][0], child[0][0]), (node[0][1], child[0][1]),
                        'k-', linewidth=0.1)
                #ax.plot(child[0][0], child[0][1], 'y*', markersize=5, label='point')
                child_list.append((child[0][0], child[0][1]))
                weight_list.append(child[1])

        # plot according to weights!
        plt.scatter([x[0] for x in child_list], [x[1] for x in child_list],
                    c=weight_list, cmap='YlOrRd')



    # similar to plot graph but I used this one for testing. can probably be
    # deleted to be honest
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


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.subscription = \
            self.create_subscription(ArucoDetection,\
            'aruco_detections',
            self.listener_callback,
            10)
        self.subscription
        self.publisher_ = self.create_publisher(Twist, 'jetbot/cmd_vel', 10)

    #aruco_opencv_msgs.msg.MarkerPose(
    #    marker_id=0,
    #    pose=geometry_msgs.msg.Pose(
    #        position=geometry_msgs.msg.Point(
    #            x=0.04391417428470471,
    #            y=0.07088609119679745,
    #            z=0.34099726260867325),
    #        orientation=geometry_msgs.msg.Quaternion(
    #            x=0.7078894961373547,
    #            y=0.704568806655969,
    #            z=-0.02631703666722777,
    #            w=-0.04222169497831387)))
    def listener_callback(self, aruco_msg):
        global twist
        global jetbot_path
        global goal_angle

        # add points to list of markers
        mark = {}
        for i in range(len(aruco_msg.markers)):
            m_id = aruco_msg.markers[i].marker_id
            mark[aruco_msg.markers[i].marker_id] = \
                    Point(aruco_msg.markers[i].pose.position.x, \
                          aruco_msg.markers[i].pose.position.y, \
                          aruco_msg.markers[i].pose.position.z)

            #print("x["+str(m_id)+"]="+str(aruco_msg.markers[i].pose.position.x))
            #print("y["+str(m_id)+"]="+str(aruco_msg.markers[i].pose.position.y))
            #print("z["+str(m_id)+"]="+str(aruco_msg.markers[i].pose.position.z)+"\n")

        if len(mark) == 0:
            print("no marker seen")
            twist.linear.x = 0.00
            self.publisher_.publish(twist)
            return

        # robot assumed to be origin
        start = (0.0, 0.0)
        # x and z of goal, arbitray size
        goal = (mark[0].x, mark[0].z, 0.05)
        obstacles = [(mark[3].x, mark[3].z, 0.1),
                     (mark[4].x, mark[4].z, 0.1)]

        rand_seed = np.random.randint(0, 1000);
        rrt1 = RRT(start, goal, obstacles, quit_on_goal=True,
                   seeded=True, seed=rand_seed,
                   neighborhood_radius=0.20)

        rrt2 = RRT(start, goal, obstacles, quit_on_goal=False,
                   seeded=True, seed=rand_seed, iterations=500,
                   neighborhood_radius=0.20)

        rrt3 = RRT(start, goal, obstacles, quit_on_goal=False,
                   seeded=True, seed=rand_seed, iterations=1000,
                   neighborhood_radius=0.20)

        #rrt1.plan()
        rrt2.plan()
        #rrt3.plan()
        jetbot_path = rrt2.path
        #print(mark)
        #print(goal)
        #print(obstacles)
        #fig, ax = plt.subplots()
        #rrt.plot_obstacles(ax)
        #rrt.plot_path(ax, rrt.path)
        #rrt.plot_graph(ax, rrt.graph)
        plt.show()

        goal_angle = np.arctan2(mark[0].x, mark[0].z)
        print(np.rad2deg(goal_angle))

        return

def turn_k_degrees(k, pub_node):
    twist.linear.x = 0.00
    if k > 0:
        twist.angular.z = 1.18
    if k < 0:
        twist.angular.z = -1.18
    pub_node.publisher_.publish(twist)

    time.sleep(0.017*abs(k))

    twist.angular.z = 0.0
    pub_node.publisher_.publish(twist)

def move_k_centimeters(k, pub_node):
    twist.angular.z = 0.0120
    if k > 0:
        twist.linear.x = -0.07
    if k < 0:
        twist.linear.x = 0.07
    pub_node.publisher_.publish(twist)

    time.sleep(0.091*abs(k))

    twist.angular.z = 0.0
    twist.linear.x = 0.0
    pub_node.publisher_.publish(twist)

def follow_path(path, pub_node):
    for angle, dist in path:
        turn_k_degrees(-angle, pub_node)
        move_k_centimeters(dist*100, pub_node)

def calculate_robot_path(points, start_to_goal_angle):
    angles_and_distances = []
    # start looking at the goal
    current_angle = np.deg2rad(90)

    for i in range(len(points) - 1):
        point_a = points[i]
        point_b = points[i + 1]

        # Check and convert point_a and point_b to tuples if they are NumPy arrays
        if isinstance(point_a, np.ndarray):
            point_a = tuple(point_a)
        if isinstance(point_b, np.ndarray):
            point_b = tuple(point_b)

        # Calculate the angle to the next point
        next_angle = math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])
        relative_angle = next_angle - current_angle

        # Convert relative angle to degrees and ensure it is between -180 and 180 degrees
        relative_angle_degrees = math.degrees(relative_angle) % 360
        if relative_angle_degrees > 180:
            relative_angle_degrees -= 360

        # Calculate the distance to the next point
        distance = math.sqrt((point_b[0] - point_a[0])**2 + (point_b[1] - point_a[1])**2)

        angles_and_distances.append((relative_angle_degrees, distance))

        current_angle = next_angle

    return angles_and_distances

def main(args=None):
        global jetbot_path

        rclpy.init(args=args)
        minimal_node = MinimalNode()
        try:
            rclpy.spin_once(minimal_node)
            # move in a square

            print('working?')
            print(goal_angle)
            turn_k_degrees(np.rad2deg(goal_angle), minimal_node)
            s_to_g_angle = np.rad2deg(np.arctan2(jetbot_path[0][0], jetbot_path[0][1]))
            print('s_to_g_angle', s_to_g_angle)
            jetbot_path.reverse()
            angle_and_dist = calculate_robot_path(jetbot_path, s_to_g_angle)
            follow_path(angle_and_dist, minimal_node)

            #move_k_centimeters(30, minimal_node)
            #turn_k_degrees(90, minimal_node)
            #move_k_centimeters(30, minimal_node)
            #turn_k_degrees(90, minimal_node)
            #move_k_centimeters(30, minimal_node)
            #turn_k_degrees(90, minimal_node)
            #move_k_centimeters(30, minimal_node)
            #turn_k_degrees(90, minimal_node)

        except KeyboardInterrupt:
            print("\nstopping...")
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            minimal_node.publisher_.publish(twist)
            sys.exit(0)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
