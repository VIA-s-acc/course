
import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            """
            Initializes an instance of the Node class.

            Parameters:
                x (float): The x-coordinate of the node.
                y (float): The y-coordinate of the node.

            Returns:
                None
            """
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            """
            Initializes an instance of the AreaBounds class.

            Parameters:
                area (list): A list containing the minimum and maximum x and y values
                            of the area.

            Returns:
                None
            """
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])


    def __init__(self,
                start,
                goal,
                obstacle_list,
                rand_area,
                expand_dis=3.0,
                path_resolution=0.5,
                goal_sample_rate=5,
                max_iter=500,
                play_area=None,
                robot_radius=0.0,
                ):
        """
        Initializes an instance of the RRT class.

        Parameters:
            start (list): The starting position of the robot as a list of [x, y] coordinates.
            goal (list): The goal position of the robot as a list of [x, y] coordinates.
            obstacle_list (list): A list of obstacles represented as lists of [x, y, size] coordinates.
            rand_area (list): A list containing the minimum and maximum values for random sampling.
            expand_dis (float, optional): The distance to expand the nodes by. Defaults to 3.0.
            path_resolution (float, optional): The resolution of the path. Defaults to 0.5.
            goal_sample_rate (int, optional): The rate at which to sample the goal. Defaults to 5.
            max_iter (int, optional): The maximum number of iterations. Defaults to 500.
            play_area (list, optional): The boundaries of the play area as a list of [xmin, xmax, ymin, ymax]. Defaults to None.
            robot_radius (float, optional): The radius of the robot. Defaults to 0.0.
        """
        
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        """
        Plans a path from the start node to the end node using the RRT algorithm.
        
        Parameters:
            animation (bool): If True, the function will display an animation of the planning process. Default is True.
        
        Returns:
            None: If no path is found.
            list: A list of nodes representing the planned path.
        """
        
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        """
        Steers the path from the given starting node to the target node.

        Args:
            from_node (Node): The starting node.
            to_node (Node): The target node.
            extend_length (float, optional): The maximum length to extend the path. Defaults to float("inf").

        Returns:
            Node: The new node after steering.

        Description:
            This function creates a new node at the starting position of the starting node. It then calculates the distance and angle between the new node and the target node. The function then extends the path from the starting node to the target node by moving the new node along the angle for a certain number of steps. The function checks if the extended path goes outside the play area or collides with any obstacles. If the extended path is within the play area and does not collide with any obstacles, the function adds the target node's position to the path and updates the new node's position to the target node's position. Finally, the function sets the parent of the new node to the starting node and returns the new node.
        """
   
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        """
        Generates the final course from the goal index.

        Parameters:
            goal_ind (int): The index of the goal node in the node_list.

        Returns:
            list: A list of lists representing the final course from the goal node to the start node.
                Each inner list contains the x and y coordinates of a node in the path.
        """
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        """
        Calculate the Euclidean distance between the given point (x, y) and the goal point.

        Parameters:
            x (float): The x-coordinate of the point.
            y (float): The y-coordinate of the point.

        Returns:
            float: The Euclidean distance between the point and the goal point.
        """
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        """
        Generate a random node within the specified range.

        This function generates a random node by sampling a point within the specified range. The range is defined by `min_rand` and `max_rand`. If the random number generated is greater than `goal_sample_rate`, a random point within the range is generated. Otherwise, the goal point is sampled.

        Returns:
            Node: A randomly generated node.
        """
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        """
        Draw the graph of the RRT algorithm.

        This function plots the graph of the RRT algorithm. It takes an optional parameter `rnd`, which is the random node to be plotted. If `rnd` is provided, the function plots the coordinates of the node as a red cross. If the robot radius is greater than 0, the function also plots a circle representing the robot's body.

        The function then plots the paths of the nodes that have a parent. The paths are plotted in green.

        Next, the function plots the obstacles in the graph. Each obstacle is represented by a circle with the coordinates `ox`, `oy`, and `size`.

        If a play area is defined, the function plots the boundaries of the play area. The boundaries are represented by a series of lines.

        Finally, the function plots the start and end points of the RRT algorithm. The start point is represented by a red cross, and the end point is also represented by a red cross.

        The function also sets the axis limits and grid settings for the plot.

        Parameters:
            rnd (Node): The random node to be plotted. Default is None.
        """
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.play_area.xmin, self.play_area.xmax, self.play_area.ymin, self.play_area.ymax])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        """
        Plot a circle with center coordinates (x, y), a given size, and a specified color.
    
        Parameters:
                x (float): x-coordinate of the circle center.
                y (float): y-coordinate of the circle center.
                size (float): radius of the circle.
                color (str, optional): color of the circle. Defaults to "-b".
        """
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        """
        Find the index of the nearest node in a list of nodes to a given random node.

        Parameters:
            node_list (List[Node]): A list of Node objects representing the nodes.
            rnd_node (Node): The random node to find the nearest neighbor for.

        Returns:
            int: The index of the nearest node in the list.
        """

        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):
        """
        Check if a given node is outside the play area.

        This static method takes a `node` object and a `play_area` object as parameters and checks if the node is outside the play area. If the `play_area` is `None`, it returns `True` indicating that every position should be considered valid. If the node's x-coordinate is less than the `xmin` value of the play area or greater than the `xmax` value, or if the node's y-coordinate is less than the `ymin` value of the play area or greater than the `ymax` value, then it returns `False` indicating that the node is outside the play area. Otherwise, it returns `True` indicating that the node is inside the play area.

        Parameters:
            node (Node): The node object to check.
            play_area (AreaBounds or None): The play area object to check against.

        Returns:
            bool: `True` if the node is inside the play area, `False` otherwise.
        """

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):
        """
        Check if a given node collides with any obstacle in the obstacle list.

        This static method takes a `node` object, an `obstacleList` list of obstacles, and a `robot_radius` as parameters. It checks if the given `node` collides with any obstacle in the `obstacleList`. If the `node` is `None`, it returns `False` indicating that there is no collision. Otherwise, it iterates through each obstacle in the `obstacleList` and calculates the distance between the `node` and the obstacle. If the minimum distance is less than or equal to the sum of the obstacle's size and the `robot_radius`, it returns `False` indicating a collision. If no collision is found, it returns `True` indicating that there is no collision.

        Parameters:
            node (Node): The node object to check for collision.
            obstacleList (List[Tuple[float, float, float]]): A list of obstacles, where each obstacle is represented as a tuple of (x, y, size).
            robot_radius (float): The radius of the robot.

        Returns:
            bool: `True` if there is no collision, `False` if there is a collision.
        """

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        """
        Calculate the distance and angle between two nodes.

        Parameters:
            from_node (Node): The starting node.
            to_node (Node): The target node.

        Returns:
            tuple: A tuple containing the distance (float) and angle (float) between the nodes.
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    obstacleList = []
    oy = [2, 3, 10, 11, 12, 2, 3, 6, 7, 8, 10, 11, 12, 6, 7, 8, 10, 11, 12, 10, 11, 12, 13, 14, 15, 13, 14, 15, 2, 3, 4, 2, 3, 4, 10, 11, 10, 11, 5, 6, 5, 6, 10, 11, 10, 11]
    px = [2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 10, 10, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14]
    for x,y in zip(px,oy):
        obstacleList.append((x, y, 4*0.125))
    # Set Initial parameters
    rrt = RRT(
        start=[9, 13],
        goal=[5, 0],
        expand_dis=1,
        rand_area=[0, 15],
        obstacle_list=obstacleList,
        play_area=[-1, 15, -1, 15],
        robot_radius=0.3
        )
    path = rrt.planning(animation=show_animation)
    print(path)
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01) 
            plt.show()


if __name__ == '__main__':
    main()