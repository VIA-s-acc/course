from controller import Robot, Supervisor, Node, Field
import matplotlib.pyplot as plt
import math, sys, struct
from Other.RRT import RRT
from Other.Astar import AStarPlanner
import numpy as np
from Other.config import show_animation as anim_flag

show_animation = anim_flag

def shortest_rotation(current_angle, target_angle):
    # Нормализация углов к диапазону [0, 2*pi)
    current_angle = current_angle % (2 * math.pi)
    target_angle = target_angle % (2 * math.pi)
    
    diff = (target_angle - current_angle + 2 * math.pi) % (2 * math.pi)
    
    if diff <= math.pi:
        return "CC"
    else:
        return "C"
# алгоритм брезенхэма
def fill_matrix_between_points(matrix, i_start, j_start, i_end, j_end):
    n = len(matrix)
    if not (0 <= i_start < n and 0 <= j_start < n):
        raise ValueError("Начальная точка вне границ матрицы")
    if not (0 <= i_end < n and 0 <= j_end < n):
        raise ValueError("Конечная точка вне границ матрицы")
    
    dx = abs(i_end - i_start)
    dy = abs(j_end - j_start)
    sx = 1 if i_start < i_end else -1
    sy = 1 if j_start < j_end else -1
    err = dx - dy
    
    while True:
        matrix[i_start][j_start] = -1
        
        if i_start == i_end and j_start == j_end:
            break
        
        e2 = 2 * err
        
        if e2 > -dy:
            err -= dy
            i_start += sx
        
        if e2 < dx:
            err += dx
            j_start += sy

    return matrix

class Enumerate(object):
    def __init__(self, names):
        for number, name in enumerate(names.split()):
            setattr(self, name, number)



class Robot_Controller(Supervisor):

    def __init__(self, algorithm="RRT"):
        super(Robot_Controller, self).__init__()
        self.algorithm = algorithm
        if self.algorithm == "RRT":
            self.multiply = 4
        if self.algorithm == "Astar":
            self.multiply = 4
        self.num_of_boxes = 4
        self.robot = self.getFromDef("Epuck")
        self.emitter = self.getDevice("emitter")
        self.Arena = self.getFromDef("RArena")
        self.Box1 = self.getFromDef("BOX1")
        self.Box2 = self.getFromDef("BOX2")
        self.Box3 = self.getFromDef("BOX3")
        self.Box4 = self.getFromDef("BOX4")
        self.Box5 = self.getFromDef("BOX5")
        self.Box6 = self.getFromDef("BOX6")
        self.Box7 = self.getFromDef("BOX7")
        self.Box8 = self.getFromDef("BOX8")
        self.Goal = self.getFromDef("GOAL")
        self.motors = []
        self.ArenaSize = self.Arena.getField("floorSize").getSFVec2f()
        self.CellSize = self.Arena.getField("floorTileSize").getSFVec2f() 
        self.CellSize[0], self.CellSize[1] = self.CellSize[0] / self.multiply, self.CellSize[1] / self.multiply 
        self.Cells_Per_Arena = int((self.ArenaSize[0] / self.CellSize[0]) * 2)
        self.Grid = [[0] * self.Cells_Per_Arena for i in range(self.Cells_Per_Arena)]
        robot_x, robot_y, _ = self.robot.getField("translation").getSFVec3f()
        goal_x, goal_y, _ = self.Goal.getField("translation").getSFVec3f()
        self.Robot_cells = int(
            math.floor((robot_x * self.Cells_Per_Arena) + (self.Cells_Per_Arena / 2))
        ), int(
            math.floor((robot_y * self.Cells_Per_Arena) + (self.Cells_Per_Arena / 2))
        )
        self.Goal_cells = int(
            math.floor((goal_x * self.Cells_Per_Arena) + (self.Cells_Per_Arena / 2))
        ), int(math.floor((goal_y * self.Cells_Per_Arena) + (self.Cells_Per_Arena / 2)))
        # print("Area Size:", self.ArenaSize, "\nCell Size:", self.CellSize, '\nRobot and Goal cells:', self.Robot_cells, self.Goal_cells, '\nGrid Size:' + str(self.Cells_Per_Arena) + 'x' + str(self.Cells_Per_Arena), "\nEmitter:", self.emitter, "\nRobot:", self.robot, '\nGrid:\n', np.array(self.Grid))

        box1_x, box1_y, _ = self.Box1.getField("translation").getSFVec3f()
        box2_x, box2_y, _ = self.Box2.getField("translation").getSFVec3f()
        box3_x, box3_y, _ = self.Box3.getField("translation").getSFVec3f()
        box4_x, box4_y, _ = self.Box4.getField("translation").getSFVec3f()
        box5_x, box5_y, _ = self.Box5.getField("translation").getSFVec3f()
        box6_x, box6_y, _ = self.Box6.getField("translation").getSFVec3f()
        box7_x, box7_y, _ = self.Box7.getField("translation").getSFVec3f()
        box8_x, box8_y, _ = self.Box8.getField("translation").getSFVec3f()
        box1_x_size, box1_y_size, _ = self.Box1.getField("size").getSFVec3f()
        box2_x_size, box2_y_size, _ = self.Box2.getField("size").getSFVec3f()
        box3_x_size, box3_y_size, _ = self.Box3.getField("size").getSFVec3f()
        box4_x_size, box4_y_size, _ = self.Box4.getField("size").getSFVec3f()
        box5_x_size, box5_y_size, _ = self.Box5.getField("size").getSFVec3f()
        box6_x_size, box6_y_size, _ = self.Box6.getField("size").getSFVec3f()
        box7_x_size, box7_y_size, _ = self.Box7.getField("size").getSFVec3f()
        box8_x_size, box8_y_size, _ = self.Box8.getField("size").getSFVec3f()
        _,_, z1 ,box1_theta = self.Box1.getField("rotation").getSFRotation()
        _,_, z2, box2_theta = self.Box2.getField("rotation").getSFRotation()
        _,_, z3, box3_theta = self.Box3.getField("rotation").getSFRotation()
        _,_, z4, box4_theta = self.Box4.getField("rotation").getSFRotation()
        _,_, z5, box5_theta = self.Box5.getField("rotation").getSFRotation()
        _,_, z6, box6_theta = self.Box6.getField("rotation").getSFRotation()
        _,_, z7, box7_theta = self.Box7.getField("rotation").getSFRotation()
        _,_, z8, box8_theta = self.Box8.getField("rotation").getSFRotation()

        box1_theta, box2_theta, box3_theta, box4_theta, box5_theta, box6_theta, box7_theta, box8_theta = box1_theta*z1, box2_theta*z2, box3_theta*z3, box4_theta*z4, box5_theta*z5, box6_theta*z6, box7_theta*z7, box8_theta*z8
        # up left, up, right, bottom left, bottom right
        box1_x_corner = [
            box1_x - (box1_x_size / 2 - 0.01),
            box1_x + (box1_x_size / 2 - 0.01),
            box1_x - (box1_x_size / 2 - 0.01),
            box1_x + (box1_x_size / 2 - 0.01),
        ]
        box1_y_corner = [
            box1_y + (box1_y_size / 2 - 0.01),
            box1_y + (box1_y_size / 2  - 0.01),
            box1_y - (box1_y_size / 2  - 0.01),
            box1_y - (box1_y_size / 2  - 0.01),
        ]
        box2_x_corner = [
            box2_x - (box2_x_size / 2  - 0.01),
            box2_x + (box2_x_size / 2  - 0.01),
            box2_x - (box2_x_size / 2  - 0.01),
            box2_x + (box2_x_size / 2  - 0.01),
        ]
        box2_y_corner = [
            box2_y + (box2_y_size / 2  - 0.01),
            box2_y + (box2_y_size / 2  - 0.01),
            box2_y - (box2_y_size / 2  - 0.01),
            box2_y - (box2_y_size / 2  - 0.01),
        ]
        box3_x_corner = [
            box3_x - (box3_x_size / 2  - 0.01),
            box3_x + (box3_x_size / 2  - 0.01),
            box3_x - (box3_x_size / 2  - 0.01),
            box3_x + (box3_x_size / 2  - 0.01),
        ]
        box3_y_corner = [
            box3_y + (box3_y_size / 2  - 0.01),
            box3_y + (box3_y_size / 2  - 0.01),
            box3_y - (box3_y_size / 2  - 0.01),
            box3_y - (box3_y_size / 2  - 0.01),
        ]
        box4_x_corner = [
            box4_x - (box4_x_size / 2 - 0.01),
            box4_x + (box4_x_size / 2 - 0.01),
            box4_x - (box4_x_size / 2 - 0.01),
            box4_x + (box4_x_size / 2 - 0.01),
        ]
        box4_y_corner = [
            box4_y + (box4_y_size / 2 - 0.01),
            box4_y + (box4_y_size / 2 - 0.01),
            box4_y - (box4_y_size / 2 - 0.01),
            box4_y - (box4_y_size / 2 - 0.01),
        ]
        box5_x_corner = [
            box5_x - (box5_x_size / 2 - 0.01),
            box5_x + (box5_x_size / 2 - 0.01),
            box5_x - (box5_x_size / 2 - 0.01),
            box5_x + (box5_x_size / 2 - 0.01),
        ]
        box5_y_corner = [
            box5_y + (box5_y_size / 2 - 0.01),
            box5_y + (box5_y_size / 2 - 0.01),
            box5_y - (box5_y_size / 2 - 0.01),
            box5_y - (box5_y_size / 2 - 0.01),
        ]
        box6_x_corner = [
            box6_x - (box6_x_size / 2 - 0.01),
            box6_x + (box6_x_size / 2 - 0.01),
            box6_x - (box6_x_size / 2 - 0.01),
            box6_x + (box6_x_size / 2 - 0.01),
        ]
        box6_y_corner = [
            box6_y + (box6_y_size / 2 - 0.01),
            box6_y + (box6_y_size / 2 - 0.01),
            box6_y - (box6_y_size / 2 - 0.01),
            box6_y - (box6_y_size / 2 - 0.01),
        ]
        box7_x_corner = [
            box7_x - (box7_x_size / 2 - 0.01),
            box7_x + (box7_x_size / 2 - 0.01),
            box7_x - (box7_x_size / 2 - 0.01),
            box7_x + (box7_x_size / 2 - 0.01),
        ]
        box7_y_corner = [
            box7_y + (box7_y_size / 2 - 0.01),
            box7_y + (box7_y_size / 2 - 0.01),
            box7_y - (box7_y_size / 2 - 0.01),
            box7_y - (box7_y_size / 2 - 0.01),
        ]
        box8_x_corner = [
            box8_x - (box8_x_size / 2 - 0.01),
            box8_x + (box8_x_size / 2 - 0.01),
            box8_x - (box8_x_size / 2 - 0.01),
            box8_x + (box8_x_size / 2 - 0.01),
        ]
        box8_y_corner = [
            box8_y + (box8_y_size / 2 - 0.01),
            box8_y + (box8_y_size / 2 - 0.01),
            box8_y - (box8_y_size / 2 - 0.01),
            box8_y - (box8_y_size / 2 - 0.01),
        ]
        def rotate(x,y,theta,nb, center_x, center_y):
            for i in range(0,nb):
                x[i] = x[i] - center_x
                y[i] = y[i] - center_y
                x_rotated = x[i] * math.cos(theta) - y[i] * math.sin(theta)
                y_rotated = x[i] * math.sin(theta) + y[i] * math.cos(theta)
                x[i] = x_rotated + center_x
                y[i] = y_rotated + center_y

        rotate(box1_x_corner, box1_y_corner, box1_theta, self.num_of_boxes, box1_x, box1_y)
        rotate(box2_x_corner, box2_y_corner, box2_theta, self.num_of_boxes, box2_x, box2_y)
        rotate(box3_x_corner, box3_y_corner, box3_theta, self.num_of_boxes, box3_x, box3_y)
        rotate(box4_x_corner, box4_y_corner, box4_theta, self.num_of_boxes, box4_x, box4_y)
        rotate(box5_x_corner, box5_y_corner, box5_theta, self.num_of_boxes, box5_x, box5_y)
        rotate(box6_x_corner, box6_y_corner, box6_theta, self.num_of_boxes, box6_x, box6_y)
        rotate(box7_x_corner, box7_y_corner, box7_theta, self.num_of_boxes, box7_x, box7_y)
        rotate(box8_x_corner, box8_y_corner, box8_theta, self.num_of_boxes, box8_x, box8_y)
        Boxes_x = []
        Boxes_y = []
        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box1_x_corner[i])
            Boxes_y.append(box1_y_corner[i])

        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box2_x_corner[i])
            Boxes_y.append(box2_y_corner[i])

        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box3_x_corner[i])
            Boxes_y.append(box3_y_corner[i])

        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box4_x_corner[i])
            Boxes_y.append(box4_y_corner[i])

        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box5_x_corner[i])
            Boxes_y.append(box5_y_corner[i])

        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box6_x_corner[i])
            Boxes_y.append(box6_y_corner[i])

        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box7_x_corner[i])
            Boxes_y.append(box7_y_corner[i])

        for i in range(0, self.num_of_boxes):
            Boxes_x.append(box8_x_corner[i])
            Boxes_y.append(box8_y_corner[i])

        for i in range(0, len(Boxes_y)):
            Boxes_x[i] = int(
                math.floor(
                    (Boxes_x[i] * self.Cells_Per_Arena) + (self.Cells_Per_Arena / 2)
                )
            )
            Boxes_y[i] = int(
                math.floor(
                    (Boxes_y[i] * self.Cells_Per_Arena) + (self.Cells_Per_Arena / 2)
                )
            )
        BOXES_UP_LEFT = []
        for i in range(0, len(Boxes_x), 4):
            BOXES_UP_LEFT.append([Boxes_x[i], Boxes_y[i]])
        BOXES_UP_RIGHT  = []
        for i in range(1, len(Boxes_x), 4):
            BOXES_UP_RIGHT.append([Boxes_x[i], Boxes_y[i]])
        BOXES_DOWN_LEFT = []
        for i in range(2, len(Boxes_x), 4):
            BOXES_DOWN_LEFT.append([Boxes_x[i], Boxes_y[i]])
        BOXES_DOWN_RIGHT = []
        for i in range(3, len(Boxes_x), 4):
            BOXES_DOWN_RIGHT.append([Boxes_x[i], Boxes_y[i]])
        BOXES_DOWN_RIGHT = []
        for i in range(3, len(Boxes_x), 4):
            BOXES_DOWN_RIGHT.append([Boxes_x[i], Boxes_y[i]])

        for i in range(0, len(BOXES_UP_LEFT)):
            fill_matrix_between_points(self.Grid, BOXES_UP_LEFT[i][0], BOXES_UP_LEFT[i][1], BOXES_UP_RIGHT[i][0], BOXES_UP_RIGHT[i][1])
            fill_matrix_between_points(self.Grid, BOXES_DOWN_LEFT[i][0], BOXES_DOWN_LEFT[i][1], BOXES_DOWN_RIGHT[i][0], BOXES_DOWN_RIGHT[i][1])
            fill_matrix_between_points(self.Grid, BOXES_UP_LEFT[i][0], BOXES_UP_LEFT[i][1], BOXES_DOWN_LEFT[i][0], BOXES_DOWN_LEFT[i][1])
            fill_matrix_between_points(self.Grid, BOXES_UP_RIGHT[i][0], BOXES_UP_RIGHT[i][1], BOXES_DOWN_RIGHT[i][0], BOXES_DOWN_RIGHT[i][1])
    

        self.Grid[self.Robot_cells[0]][self.Robot_cells[1]] = 2
        self.Grid[self.Goal_cells[0]][self.Goal_cells[1]] = 4  
        Boxes_x = []
        Boxes_y = []
        # print("Grid after filling:\n")
        # max_length = max(len(str(num)) for row in self.Grid for num in row)
        # for row in self.Grid:
        #     for num in row:
        #         if num == -1:
        #             print('\033[91m' + str(num).rjust(max_length) + '\033[0m', end=" ")
        #         else:
        #             print(str(num).rjust(max_length), end=" ")
        #     print()

        for row in range(len(self.Grid)):
            for col in range(len(self.Grid[0])):
                if self.Grid[row][col] == -1:
                    Boxes_x.append(row)
                    Boxes_y.append(col)
        
        # str1 = ""
        # for row in self.Grid:
        #     str1 += " ".join("{:>3}".format(str(elem)) for elem in row) + "\n"

        if self.algorithm == "Astar":
            ox, oy = [], []
            grid_size = 1
            for x,y in (zip(Boxes_x, Boxes_y)):
                ox.append(x)
                oy.append(y)
            for i in range(-grid_size * self.multiply, self.Cells_Per_Arena+grid_size * self.multiply):
                ox.append(i)
                oy.append(-grid_size * self.multiply)
                ox.append(-grid_size * self.multiply)
                oy.append(i)
                ox.append(self.Cells_Per_Arena+grid_size * self.multiply)
                oy.append(i)
                ox.append(i)
                oy.append(self.Cells_Per_Arena+grid_size * self.multiply)
                ox.append(-grid_size * self.multiply)
                oy.append(self.Cells_Per_Arena+grid_size * self.multiply)
                ox.append(self.Cells_Per_Arena+grid_size * self.multiply)
                oy.append(-grid_size * self.multiply)
            
            sx = self.Robot_cells[0]
            sy = self.Robot_cells[1]
            gx = self.Goal_cells[0]
            gy = self.Goal_cells[1]
            
            robot_radius = self.multiply * grid_size
            a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
            
            if show_animation:  
                plt.plot(ox, oy, ".k")
                plt.plot(sx, sy, "og")
                plt.plot(gx, gy, "xb")
                plt.grid(True)
                plt.axis("equal")
            trajectory = []


            rx, ry = a_star.planning(sx, sy, gx, gy)

            rx.reverse()
            ry.reverse()
            print("Trajectory_i: ",rx)
            print("Trajectory_j: ",ry)
            for index in range(1, len(rx)):
                trajectory.append(math.ceil(rx[index]))
                trajectory.append(math.ceil(ry[index]))

            if show_animation: 
                plt.plot(rx, ry, "-r")
                plt.pause(0.001)
                plt.show()

        elif self.algorithm == "RRT":
            obstacle_list = []
            for x,y in (zip(Boxes_x, Boxes_y)):
                obstacle_list.append((x, y, self.CellSize[0]*self.multiply*4))
            if self.multiply == 4:
                mult = 2 ** (self.multiply+2) * 1.4
            elif self.multiply == 8:
                mult = 2 ** (self.multiply) * 1.25

   
            rrt = RRT(
                start = [self.Robot_cells[0], self.Robot_cells[1]],
                goal = [self.Goal_cells[0], self.Goal_cells[1]],
                obstacle_list = obstacle_list,
                goal_sample_rate = self.multiply**2,
                rand_area = [0, self.Cells_Per_Arena],
                play_area = [-1, self.Cells_Per_Arena+1, -1, self.Cells_Per_Arena+1],
                robot_radius = self.CellSize[0] * mult,
                max_iter = 500
            )
            path = rrt.planning(animation=show_animation)

            if show_animation:
                rrt.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.01)  
                plt.show()
            path.reverse()

            trajectory = []

            for index in range(1, len(path)):
                trajectory.append(math.ceil(path[index][0]))
                trajectory.append(math.ceil(path[index][1]))
                
        trajectory_x = [self.Robot_cells[0]]
        trajectory_y = [self.Robot_cells[1]]
        for i in range(len(trajectory)):
            if i % 2 == 0:
                trajectory_x.append(trajectory[i])
            else:
                trajectory_y.append(trajectory[i])

        coords_x = []
        coords_y = []
        

        for i in range(len(trajectory_x)):
            desired_x = (
                (trajectory_x[i] - float(self.Cells_Per_Arena / 2))
                / self.Cells_Per_Arena
            ) + (self.CellSize[0] / 4)
            desired_y = (
                (trajectory_y[i] - float(self.Cells_Per_Arena / 2))
                / self.Cells_Per_Arena
            ) + (self.CellSize[0] / 4)
            coords_x.append(desired_x)
            coords_y.append(desired_y)


        directions = []

        for i in range(len(trajectory_x) - 1):
            dot1 = (trajectory_x[i], trajectory_y[i])
            dot2 = (trajectory_x[i+1], trajectory_y[i+1])
            angle = math.atan2(dot2[1]-dot1[1], dot2[0]-dot1[0])
    
            directions.append(angle)
            
        for i,k,l,b,x in zip(coords_x,coords_y, trajectory_x, trajectory_y, directions):
            print((i,k), (l,b), (x))
        dist = 0
        robot_prev_post = self.robot.getField('translation').getSFVec3f()[:2]
        counter = 0
        want_turn = False
        while True:
            if self.step(64) == -1:
                    break  
            

            if counter < len(coords_x):      
            
                robot_pos_x, robot_pos_y, _ = self.robot.getField('translation').getSFVec3f()
                dist += math.sqrt(pow(robot_pos_x - robot_prev_post[0], 2) + pow(robot_pos_y - robot_prev_post[1], 2))
                robot_prev_post = robot_pos_x, robot_pos_y
                _, _, z, robot_theta = self.robot.getField('rotation').getSFRotation()                   
                
                if math.sqrt(pow(robot_pos_x - goal_x, 2) + pow(robot_pos_y - goal_y, 2)) < 0.065:
                    self.emitter.send("S")
                    print("DONE | Distance to goal: ", math.sqrt(pow(robot_pos_x - goal_x, 2) + pow(robot_pos_y - goal_y, 2)))
                    print("DONE | Distance travelled: ", dist)
                    import json
                    import os 
                    # if self.algorithm == "RRT":
                    #     if os.path.exists('RRT.json'):
                    #         distance_dict = json.load(open('RRT.json'))
                    #         distance_dict[dist] = dist
                    #         print(len(distance_dict))
                    #         first_10 = []
                    #         fisrt_25 = []
                    #         first_50  = []
                    #         first_100 = []
                    #         for enum, item in enumerate(distance_dict):
                    #             print(item)
                    #             if enum < 10:
                    #                 first_10.append(float(item))
                    #             if enum < 25:
                    #                 fisrt_25.append(float(item))
                    #             if enum < 50:
                    #                 first_50.append(float(item))
                    #             if enum < 100:
                    #                 first_100.append(float(item))
                    #         print("max 10: ", max(first_10))
                    #         print("max 25: ", max(fisrt_25))
                    #         print("max 50: ", max(first_50))
                    #         print("max 100: ", max(first_100))
                    #         print("min 10: ", min(first_10))
                    #         print("min 25: ", min(fisrt_25))
                    #         print("min 50: ", min(first_50))
                    #         print("min 100: ", min(first_100))
                    #         print("mean 10: ", sum(first_10) / len(first_10))
                    #         print("mean 25: ", sum(fisrt_25) / len(fisrt_25))
                    #         print("mean 50: ", sum(first_50) / len(first_50))
                    #         print("mean 100: ", sum(first_100) / len(first_100))

                            # with open('RRT.json', 'w') as f:
                            #     json.dump(distance_dict, f)
                        # else:
                        #     distance_dict = {dist: dist}
                        #     with open('RRT.json', 'w') as f:
                        #         json.dump(distance_dict, f)
                                                
                    break
                if (want_turn == False):    
              
                    if (math.sqrt(pow(robot_pos_x - coords_x[counter], 2)) < 0.008) and \
                        (math.sqrt(pow(robot_pos_y - coords_y[counter], 2)) < 0.008):  
                        # print('1')    
                        want_turn = True
                        order = 'S' 
                        continue 

                    else:
                        order = 'F' 
            
            
                elif (want_turn == True):
                    z = 1 if z > 0 else -1
                    # # print(int(z))
                    # # print(directions[counter])
                    # # print(robot_theta)
                    # print(abs(robot_theta * z - directions[counter]))
                    if abs(robot_theta * z - directions[counter]) > 0.05:

       
                        order = shortest_rotation(robot_theta * z , directions[counter])
            
                
                    else:                
                        want_turn = False
                        order = 'S'   
                    
                        counter = counter + 1  
                        continue 
                
                
                self.emitter.send(order)
           
   
controller = Robot_Controller("RRT")
