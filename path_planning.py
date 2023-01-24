import numpy as np
import matplotlib.pyplot as plt
import math
from sympy import symbols, Eq, solve
class path_planning:
    def __init__(self, starting_pt, finishing_pt):
        self.map = [(10, 10), (30, 10), (50, 10), (10, 30), (30, 30), (10, 50), (100, 30)]
        #self.g = Graph(5)
        #self.g.graph = [[0, 0, 0, 20, 0, 0, 0],
                        #[0, 0, 20, 0, 15, 0, 0],
                        #[0, 20, 0, 0, 0, 0, 0],
                        #[20, 0, 0, 0, 12, 80, 0],
                        #[0, 15, 0, 12, 0, 0, 30],
                        #[0, 0, 0, 80, 0, 0, 40],
                        #[0, 0, 0, 0, 30, 40, 0]]
        self.road = []
        self.completed_road = []
        self.angles = []
        self.equations = []
        self.type_equation = []  #0 if straight, 1 if corner
        self.final_trajectory = []

        #path = np.array(self.g.dijkstra(starting_pt, finishing_pt))
        path = np.array(self.djikstra_algo(starting_pt, finishing_pt))

        for i in range(len(path)):
            self.road.append(self.map[path[i]])

        self.angles_computation()
        self.complete_road()
        self.final_trajectory = self.final_list_calculator()

    def initial_graph(self):

        return {
            '0': {'3': 30},
            '1': {'2': 20, '4': 30},
            '2': {'1': 20},
            '3': {'0': 30, '4': 20, '5': 80},
            '4': {'1': 30, '3': 20, '6': 40},
            '5': {'3': 80, '6': 40},
            '6': {'5': 40, '4': 40}
        }

    def angles_computation(self):
        for i in range(len(self.road)-2):
            vector1_x = self.road[i+1][0] - self.road[i][0]
            vector1_y = self.road[i + 1][1] - self.road[i][1]
            vector2_x = self.road[i+2][0] - self.road[i+1][0]
            vector2_y = self.road[i + 2][1] - self.road[i + 1][1]
            vector1 = [vector1_x, vector1_y]
            vector2 = [vector2_x, vector2_y]

            theta = math.pi-math.acos((vector1[0]*vector2[0]+vector1[1]*vector2[1])/(math.sqrt(vector1[0]**2+vector1[1]**2)*math.sqrt(vector2[0]**2+vector2[1]**2)))
            theta = theta * 180/math.pi
            self.angles.append(round(theta,2))
        print("self.angles", self.angles)

    def complete_road(self):
        #initialization of the final road
        A = self.road[0]
        self.completed_road.append(A)
        B = self.road[1]
        # 20m = max distance for a turn
        dist_newpoint_B = round(20-0.1 * self.angles[0])
        newpoint = self.computation_position_newpoint(A, B, dist_newpoint_B)
        print("dist_newpoint_B", dist_newpoint_B)
        self.completed_road.append(newpoint)
        print("completed road point", self.completed_road)
        self.completed_road.append(B)  # we will need the corner point later
        self.type_equation.append(0)

        A = self.road[2]
        newpoint = self.computation_position_newpoint(A, B, dist_newpoint_B)
        print("dist_newpoint_B", dist_newpoint_B)
        self.completed_road.append(newpoint)
        print("completed road point", self.completed_road)
        self.type_equation.append(1)

        for i in range(1, len(self.angles)):
            for j in range(2):
                if j == 0:
                    #entry in the corner
                    A = self.road[i]
                    B = self.road[i+1]
                    dist_newpoint_B = round(20 - 0.1 * self.angles[i], 2)

                if j == 1:
                    #exit of the corner
                    A = self.road[i+2]
                    B = self.road[i+1]
                    print("dist_newpoint_B", dist_newpoint_B)
                    self.completed_road.append(B) #we will need the corner point later
                    print("completed road point", self.completed_road)


                newpoint = self.computation_position_newpoint(A, B, dist_newpoint_B)

                if j == 0 and math.dist(A, newpoint) < math.dist(A, self.completed_road[-1]):
                    #if the new point is before the last one, we replace both of them by the average point
                    #meaning 2 corners are too close
                    last_point = self.completed_road[-1]
                    self.completed_road.pop()
                    d_first_corner = math.dist(newpoint, A) + math.dist(newpoint, last_point)/2
                    last_point = self.computation_position_newpoint(B, A, d_first_corner)
                    self.completed_road.append(last_point)

                    #the point before needs also to be replaced to be at the same distance
                    temporary_A = self.road[i-1]
                    temporary_B = self.road[i]
                    self.completed_road[-3] = self.computation_position_newpoint(temporary_A, temporary_B, d_first_corner)

                    #the point after is set now, ++j because we don't need to do the exit of the corner anymore
                    future_A = self.road[i+2]
                    future_B = self.road[i+1]
                    d_second_corner = math.dist(A, B) - d_first_corner
                    self.completed_road.append(future_B)  # we will need the corner point later
                    self.completed_road.append(self.computation_position_newpoint(future_A, future_B, d_second_corner))
                    self.type_equation.append(1)
                    break

                else:
                    print("dist_newpoint_B", dist_newpoint_B)
                    self.completed_road.append(newpoint)
                    if j == 0:
                        self.type_equation.append(0)
                    else:
                        self.type_equation.append(1)

        self.type_equation.append(0)
        #final points
        self.completed_road.append(self.road[-1])
        for i in range(len(self.completed_road)):
            if isinstance(self.completed_road[i], tuple):
                self.completed_road[i] = list(self.completed_road[i])
            elif not isinstance(self.completed_road[i], list):
                self.completed_road[i] = [self.completed_road[i]]
            self.completed_road[i] = [round(self.completed_road[i][0], 2), round(self.completed_road[i][1], 2)]
        print("completed road", self.completed_road)


    def computation_position_newpoint(self, A, B, dist_newpoint_B):
        equation_line = self.parametrization_computation(A, B)
        m_x, m_y, p_x, p_y = equation_line

        t = symbols(' t ')
        eq1 = Eq((B[0] - (m_x * t + p_x)) ** 2 + (B[1] - (m_y * t + p_y)) ** 2 - dist_newpoint_B ** 2, 0)
        sol = solve(eq1)

        if sol[0] <= 1:
            solution = sol[0]
        else:
            solution = sol[1]

        newpoint = (m_x * round(solution, 4) + p_x, m_y * round(solution, 4) + p_y)
        return newpoint


    def djikstra_algo(self, starting_point, finishing_point):
        starting_point = str(starting_point)
        finishing_point = str(finishing_point)
        path = {}
        adj_node = {}
        queue = []
        final_path = []
        graph = self.initial_graph()

        for node in graph:
            path[node] = float("inf")
            adj_node[node] = None
            queue.append(node)

        path[starting_point] = 0
        while queue:
            # find min distance which wasn't marked as current
            key_min = queue[0]
            min_val = path[key_min]
            for n in range(1, len(queue)):
                if path[queue[n]] < min_val:
                    key_min = queue[n]
                    min_val = path[key_min]
            cur = key_min
            queue.remove(cur)

            for i in graph[cur]:
                alternate = graph[cur][i] + path[cur]
                if path[i] > alternate:
                    path[i] = alternate
                    adj_node[i] = cur

        x = finishing_point
        final_path.append(x)
        while True:
            x = adj_node[x]
            if x is None:
                break
            final_path.insert(0, x)

        return [int(x) for x in final_path]

    def parametrization_computation(self, point1, point2):
        m_x = point2[0] - point1[0]
        m_y = point2[1] - point1[1]
        p_x = point1[0]
        p_y = point1[1]

        return m_x, m_y, p_x, p_y

    def corner_equation(self, cornerlist, point1, point2, point3):
        # system of equations
        c1=point1[0] - point3[0]
        c2=point1[1] - point3[1]
        d1=point2[0] - point3[0]
        d2=point2[1] - point3[1]

        # A = np.array([[c1, c2], [d1, d2]])
        A = np.array([[point1[0] - point3[0], point1[1] - point3[1]], [point2[0] - point3[0], point2[1] - point3[1]]])
        b = np.array([[point1[0] ** 2 + point1[1] ** 2 - point1[0] * point3[0] - point1[1] * point3[1]],
                      [point2[0] ** 2 + point2[1] ** 2 - point2[0] * point3[0] - point2[1] * point3[1]]])

        C = np.array([[d2, -c2], [-d1, c1]])
        if (c1 * d2 - d1 * c2) != 0:
            sol = (1 / (c1 * d2 - d1 * c2)) * C.dot(b)
        else:
            print("problem")
        # sol = np.linalg.solve(A, b)
        sol = [list(x) for x in sol]
        solution = [0, 0]
        print("sol", sol)
        solution[0] = round(sol[0][0], 2)
        solution[1] = round(sol[1][0], 2)
        r = math.dist(solution, point1)  # sortir les valeurs calculées

        # Calcul angles de depart, d'arrivée, et angle de l'arc
        if (point1[0] - solution[0]) == 0:
            if (point1[1] - solution[1]) > 0:
                theta1 = math.pi / 2
            else:
                theta1 = 3 * math.pi / 2
        else:
            theta1 = math.atan2((point1[1] - solution[1]), (point1[0] - solution[0]))

        if (point2[0] - solution[0]) == 0:
            if (point2[1] - solution[1]) > 0:
                theta2 = math.pi / 2
            else:
                theta2 = 3 * math.pi / 2
        else:
            theta2 = math.atan2((point2[1] - solution[1]), (point2[0] - solution[0]))

        if theta1 == math.pi:
            if theta2 < 0:
                theta1 = -theta1

        if theta2 == math.pi:
            if theta1 < 0:
                theta2 = -theta2

        theta = theta2 - theta1

        for j in range(1, 11):  # 10 points par virage, à modifier
            t = j * theta / 10 + theta1

            x = round(solution[0] + r * math.cos(t), 2)
            y = round(solution[1] + r * math.sin(t), 2)

            point = []
            point.append(x)
            point.append(y)
            cornerlist.append(point)
        print("r", r)
        print("distance1", math.dist(point1, point3))
        print("distance2", math.dist(point2, point3))

        return cornerlist

    def final_list_calculator(self):
        final_list = []
        final_list.append(self.completed_road[0])
        counter = 0
        for i in range(len(self.type_equation)):
            if self.type_equation[i] == 0:
                for j in range(1, 11):  # 10 points per corner, can be modified
                    distance = j * math.dist(self.completed_road[counter], self.completed_road[counter + 1]) / 10
                    newpoint = self.computation_position_newpoint(self.completed_road[counter + 1],
                                                                  self.completed_road[counter],
                                                                  distance)  # inversed because we want the distance from self.completed_road[counter+1] to decrease
                    final_list.append(newpoint)

            elif self.type_equation[i] == 1:
                print("nouveau virage avec i=", i)
                print("counter=", counter)
                final_list = self.corner_equation(final_list, self.completed_road[counter],
                                                  self.completed_road[counter + 2], self.completed_road[counter + 1])
                counter += 1
            counter += 1
        print("final list ", final_list)

        for i in range(len(final_list)):
            if isinstance(final_list[i], tuple):
                final_list[i] = list(final_list[i])
            elif not isinstance(final_list[i], list):
                final_list[i] = [final_list[i]]

            final_list[i] = [round(final_list[i][0], 1), round(final_list[i][1], 1)]

        return final_list
