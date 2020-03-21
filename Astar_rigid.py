import math
import matplotlib.pyplot as plt
import time
import matplotlib.animation as anim


show_animation = True


class AStarAlgo:

    def __init__(self, obstaclex, obstacley, resolution, robot_radius, theta_s, step_size, startx, starty):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.obstacle_map(obstaclex, obstacley)
        self.motion = self.motion_mode(theta_s, step_size, startx, starty)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planner(self, startx, starty, goalx, goaly):
       

        start_node = self.Node(self.xy_index(startx, self.minx),
                           self.xy_index(starty, self.miny), 0.0, -1)
        goal_node = self.Node(self.xy_index(goalx, self.minx),
                          self.xy_index(goaly, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("No Solution Found")
                break

            current_node_index = min(open_set, key=lambda o: open_set[o].cost + self.euclidean_distance(goal_node, open_set[o]))
            current_node = open_set[current_node_index]

            if show_animation:
                plt.plot(self.grid_position(current_node.x, self.minx),
                         self.grid_position(current_node.y, self.miny), "xc")
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                print("Goal node found")
                goal_node.parent_index = current_node.parent_index
                goal_node.cost = current_node.cost
                break

            # Remove the item from the open set
            del open_set[current_node_index]

            # Add it to the closed set
            closed_set[current_node_index] = current_node

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current_node.x + self.motion[i][0],
                                 current_node.y + self.motion[i][1],
                                 current_node.cost + self.motion[i][2], current_node_index)
                node_index = self.grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if node_index in closed_set:
                    continue

                if node_index not in open_set:
                    open_set[node_index] = node  # discovered a new node
                else:
                    if open_set[node_index].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[node_index] = node

        resultx, resulty = self.final_path(goal_node, closed_set)

        return resultx, resulty

    def final_path(self, goal_node, closedset):
        # generate final course
        resultx, resulty = [self.grid_position(goal_node.x, self.minx)], [
            self.grid_position(goal_node.y, self.miny)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closedset[parent_index]
            resultx.append(self.grid_position(n.x, self.minx))
            resulty.append(self.grid_position(n.y, self.miny))
            parent_index = n.parent_index

        return resultx, resulty

    @staticmethod
    def euclidean_distance(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def grid_position(self, index, minp):

        pos = index * self.resolution + minp
        return pos

    def xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.grid_position(node.x, self.minx)
        py = self.grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def obstacle_map(self, obstaclex, obstacley):

        self.minx = round(min(obstaclex))
        self.miny = round(min(obstacley))
        self.maxx = round(max(obstaclex))
        self.maxy = round(max(obstacley))

        self.xwidth = round((self.maxx - self.minx) / self.resolution)
        self.ywidth = round((self.maxy - self.miny) / self.resolution)
  

        # obstacle map generation
        self.obstacle_map = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.grid_position(iy, self.miny)
                for iobstaclex, iobstacley in zip(obstaclex, obstacley):
                    d = math.hypot(iobstaclex - x, iobstacley - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def motion_mode(theta_s, step_size, startx, starty):



        motion = [[step_size, 0, math.sqrt(step_size)],
                [step_size, -step_size, math.sqrt(step_size)],
                [0, step_size, math.sqrt(step_size)],
                [step_size, -step_size, (math.sqrt(2)*step_size)],
                [step_size, step_size, (math.sqrt(2)*step_size)]]

        return motion


def main():

    startx, starty, theta_s = list(map(int, input("Enter the x coordinate, y coordinate of the start node and orientation of the robot separated by spaces: ").split()))
    step_size = int(input("Enter step size: "))
    robot_radius = int(input("Enter robot radius: "))
    clearance = int(input("Enter the clearance: "))
    goalx, goaly = list(map(int, input("Enter the x coordinate and y coordinate of the goal node separated by spaces: ").split()))
    grid_size = 2

    # set obstable positions
    obstaclex, obstacley = [], []
    for i in range(0, 300):
        obstaclex.append(i)
        obstacley.append(0)
    for i in range(0, 200):
        obstaclex.append(300)
        obstacley.append(i)
    for i in range(0, 301):
        obstaclex.append(i)
        obstacley.append(200)
    for i in range(0, 201):
        obstaclex.append(0)
        obstacley.append(i)
    for i in range(300):
        for j in range(200):
            if ((i - 225) ** 2 + (j - 150) ** 2 <= (25 + robot_radius + clearance) ** 2):
                obstaclex.append(i)
                obstacley.append(j)
            if (((i - 150) ** 2 / (40 + robot_radius + clearance) ** 2 + (j - 100) ** 2 / (20 + robot_radius + clearance) ** 2) <= 1):
                obstaclex.append(i)
                obstacley.append(j)
            if ((j - (0.6 * i)) >= (-125 - robot_radius - clearance) and (j - (-0.6 * i)) <= (175 + robot_radius + clearance) and (j - (0.6 * i)) <= (-95 + robot_radius + clearance) and (j - (-0.6 * i)) >= (145 - robot_radius - clearance)):
                obstaclex.append(i)
                obstacley.append(j)
            if ((j - (13 * i)) <= (-140 + robot_radius + clearance) and (j - (1 * i)) >= (100 - robot_radius - clearance) and j <= (185 + robot_radius + clearance) and j - (1.4 * i) >= (80 - robot_radius - clearance)):
                obstaclex.append(i)
                obstacley.append(j)
            if ((j - (-1.2 * i)) >= (210 - robot_radius - clearance) and (j - (1.2 * i)) >= (30 - robot_radius + clearance) and (j - (-1.4 * i)) <= (290 + robot_radius + clearance) and (j - (-2.6 * i)) >= (280 - robot_radius - clearance) and j <= (185 + robot_radius+ clearance)):
                obstaclex.append(i)
                obstacley.append(j)
            if ((j - (1.73) * i + 135 >= 0 - robot_radius - clearance) and (j + (0.58) * i - 96.35 <= 0 + robot_radius + clearance) and (j - (1.73) * i - 15.54 <= 0 + robot_radius + clearance) and (j + (0.58) * i - 84.81 >= 0 - robot_radius - clearance)):
                obstaclex.append(i)
                obstacley.append(j)

    StartTime = time.time()
    if show_animation:  # pragma: no cover
        plt.plot(obstaclex, obstacley, ".k")
        plt.plot(startx, starty, "og")
        plt.plot(goalx, goaly, "xb")
        plt.grid(False)
        plt.axis("equal")


    solver = AStarAlgo(obstaclex, obstacley, grid_size, robot_radius, theta_s, step_size, startx, starty)
    resultx, resulty = solver.planner(startx, starty, goalx, goaly)
    EndTime = time.time()
    print("Solved in:", EndTime - StartTime)
    if show_animation:
        plt.plot(resultx, resulty, "-r")
        plt.show()



if __name__ == '__main__':
    main()
