import numpy as np
# import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:

    # RRT* parameters
    start = (0., 0.)
    goal = (0., 0.)
    obstacle_list = []  # Format: (x, y, radius), the position after robot see it
    x_limit = (0., 0.)    # need the size of the map
    y_limit = (0., 0.)    # need the size of the map
    step_size = 0.5          # related to the complexity of RRT* path, the less the more complex
    max_iterations = 500
    # RRT* parameters

    def __init__(self, start, goal, obstacle_list, x_limit, y_limit, step_size=1.0, max_iterations=1000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.x_limit = x_limit
        self.y_limit = y_limit
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.node_list = [self.start]

    def generate_random_node(self):
        x = np.random.uniform(self.x_limit[0], self.x_limit[1])
        y = np.random.uniform(self.y_limit[0], self.y_limit[1])
        return Node(x, y)

    def find_nearest_node(self, node):
        distances = [(n.x - node.x) ** 2 + (n.y - node.y) ** 2 for n in self.node_list]
        nearest_index = np.argmin(distances)
        return self.node_list[nearest_index]

    def is_collision_free(self, node):
        for obstacle in self.obstacle_list:
            if (node.x - obstacle[0]) ** 2 + (node.y - obstacle[1]) ** 2 <= obstacle[2] ** 2:
                return False
        return True

    def steer(self, from_node, to_node):
        angle = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * np.cos(angle)
        new_y = from_node.y + self.step_size * np.sin(angle)
        return Node(new_x, new_y)

    def calculate_cost(self, from_node, to_node):
        return from_node.cost + np.sqrt((to_node.x - from_node.x) ** 2 + (to_node.y - from_node.y) ** 2)

    def is_goal_reachable(self, node):
        return np.sqrt((node.x - self.goal.x) ** 2 + (node.y - self.goal.y) ** 2) <= self.step_size

    def rewire(self, new_node):
        for node in self.node_list:
            if node != new_node and np.sqrt((node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2) <= self.step_size:
                new_cost = self.calculate_cost(new_node, node)
                if new_cost < node.cost:
                    node.parent = new_node
                    node.cost = new_cost

    # def bspline_interpolation(self, path, degree=3, num_points=100):
    #     # Remove duplicate points from the path to ensure strictly increasing x values
    #     unique_path = np.unique(path, axis=0)

    #     x = unique_path[:, 0]
    #     y = unique_path[:, 1]
    #     tck = make_interp_spline(x, y, k=degree)
    #     spline = lambda xi: np.array([tck(xi), tck(xi, 1)])
    #     x_new = np.linspace(min(x), max(x), num_points)
    #     y_new = spline(x_new)
    #     return x_new, y_new

    def find_path(self):
        for iteration in range(self.max_iterations):
            if iteration % 50 == 0:
                print("iteration(RRT*):", iteration)
            random_node = self.generate_random_node()
            nearest_node = self.find_nearest_node(random_node)
            new_node = self.steer(nearest_node, random_node)

            if self.is_collision_free(new_node):
                near_nodes = [node for node in self.node_list if np.sqrt((node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2) <= 2 * self.step_size]
                min_cost_node = nearest_node
                min_cost = self.calculate_cost(nearest_node, new_node)

                for near_node in near_nodes:
                    if near_node.cost + np.sqrt((near_node.x - new_node.x) ** 2 + (near_node.y - new_node.y) ** 2) < min_cost:
                        min_cost_node = near_node
                        min_cost = near_node.cost + np.sqrt((near_node.x - new_node.x) ** 2 + (near_node.y - new_node.y) ** 2)

                new_node.parent = min_cost_node
                new_node.cost = min_cost
                self.node_list.append(new_node)
                self.rewire(new_node)
                #'''
                if self.is_goal_reachable(new_node):
                    print("iteration(RRT*) find path:", iteration)
                    goal_node = Node(self.goal.x, self.goal.y)
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + np.sqrt((new_node.x - self.goal.x) ** 2 + (new_node.y - self.goal.y) ** 2)
                    self.node_list.append(goal_node)
                    # print("iteration(RRT*):", iteration)
                    return self.extract_path(goal_node)

                #'''


        ######
        # if self.is_goal_reachable(new_node):
        #     goal_node = Node(self.goal.x, self.goal.y)
        #     goal_node.parent = new_node
        #     goal_node.cost = new_node.cost + np.sqrt((new_node.x - self.goal.x) ** 2 + (new_node.y - self.goal.y) ** 2)
        #     self.node_list.append(goal_node)
        #     return self.extract_path(goal_node)
        ########

        print("none")
        return None

    def extract_path(self, goal_node):
        path = [[goal_node.x, goal_node.y]]
        current_node = goal_node

        while current_node.parent is not None:
            current_node = current_node.parent
            path.append([current_node.x, current_node.y])

        return path[::-1]

    # def plot(self, path=None):
    #     for obstacle in self.obstacle_list:
    #         circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color=plt.cm.Reds(0.5))
    #         plt.gca().add_patch(circle)
    #     for node in self.node_list:
    #         if node.parent is not None:
    #             # plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-', color=plt.cm.Greens(0.1))
    #             pass
    #     if path is not None:
    #         plt.plot([x for x, y in path], [y for x, y in path], '-b')
    #     plt.plot(self.start.x, self.start.y, 'ro', markersize=8)
    #     plt.plot(self.goal.x, self.goal.y, 'bo', markersize=8)
    #     plt.xlim(self.x_limit)
    #     plt.ylim(self.y_limit)
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #     plt.title('RRT* Path Planning')
    #     plt.grid(True)
    #     # plt.show()

    def do_RRTstar(start, goal, obstacle_list, x_limit, y_limit, step_size, max_iterations, current_robot_position):
        # global start, goal, obstacle_list, x_limit, y_limit, step_size, max_iterations
        rrt_star = RRTStar(start, goal, obstacle_list, x_limit, y_limit, step_size, max_iterations)
        path = rrt_star.find_path()
        if path is not None:
            print("Path found!")
            print(path)
            print("The robot orientation: ", current_robot_position[0][2])
            # x_sp, y_sp = rrt_star.bspline_interpolation(path)  # Get the B-spline interpolated curve
            # plt.plot(x_sp, y_sp[1], color='purple')  # Plot the B-spline curve
            # rrt_star.plot(path)
            return path
        else:
            print("Path not found.")
            return None

