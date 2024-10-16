import numpy as np
import matplotlib.pyplot as plt
import math
from sympy import symbols, Eq, solve



class path_planning:
    def __init__(self, starting_pt, finishing_pt):
        self.map = [(101.5,64.6),(419.5,25.9), (463.7,25.4), (115.7,336.2),
           (442.4,312.8), (489.9,308.4), (471.3,370.1), (139.2,673.9),
           (122.8,718.6), (910.4,701.7), (133.7,893.7), (189.3,893.1),
           (477.9,874.6), (527.5,868.6), (150.1,1128.7), (204.1,1125.5),
           (510,1232.4), (552.6,1226.4),(904.9,1189.8),(936,1159.3)]
        self.map = [(x/4.6,y/4.6) for (x,y) in self.map] # Downscale to meters
        self.road = []
        self.completed_road = []
        self.angles = []
        self.equations = []
        self.type_equation = []  #0 if straight, 1 if corner
        self.final_trajectory = []

        path = np.array(self.djikstra_algo(starting_pt, finishing_pt))

        for i in range(len(path)):
            self.road.append(self.map[path[i]])

        self.angles_computation()
        self.complete_road()
        self.final_trajectory = self.final_list_calculator()

    def visibility_graph(self):
        return {
            '0': {'3': math.dist(self.map[0],self.map[3])},
            '1': {'4': math.dist(self.map[1],self.map[4])},
            '2': {'5': math.dist(self.map[2],self.map[5])},
            '3': {'0': math.dist(self.map[3],self.map[0]), '4': math.dist(self.map[3],self.map[4]), '7': math.dist(self.map[3],self.map[7])},
            '4': {'1': math.dist(self.map[4],self.map[1]), '3': math.dist(self.map[4],self.map[3]), '5': math.dist(self.map[4],self.map[5]),'6':math.dist(self.map[4],self.map[6])},
            '5': {'2': math.dist(self.map[5],self.map[2]), '4': math.dist(self.map[5],self.map[4]),'6':math.dist(self.map[5],self.map[6])},
            '6': {'4': math.dist(self.map[6],self.map[4]), '5': math.dist(self.map[6],self.map[5]),'13':math.dist(self.map[6],self.map[13])},
            '7':{'3':math.dist(self.map[7],self.map[3]),'8':math.dist(self.map[7],self.map[8])},
            '8':{'7':math.dist(self.map[8],self.map[7]),'10':math.dist(self.map[8],self.map[10])},
            '9':{'19':math.dist(self.map[9],self.map[19])},
            '10':{'8':math.dist(self.map[10],self.map[8]),'11':math.dist(self.map[10],self.map[11]),'14':math.dist(self.map[10],self.map[14])},
            '11': {'10':math.dist(self.map[11],self.map[10]),'15':math.dist(self.map[11],self.map[15]),'12':math.dist(self.map[11],self.map[12])},
            '12': {'11':math.dist(self.map[12],self.map[11]),'13':math.dist(self.map[12],self.map[13])},
            '13': {'6':math.dist(self.map[13],self.map[6]),'12':math.dist(self.map[13],self.map[12]),'17':math.dist(self.map[13],self.map[17])},
            '14': {'10':math.dist(self.map[14],self.map[10]),'15':math.dist(self.map[14],self.map[15])},
            '15': {'11':math.dist(self.map[15],self.map[11]),'14':math.dist(self.map[15],self.map[14])},
            '16': {'12':math.dist(self.map[16],self.map[12]),'17':math.dist(self.map[16],self.map[17])},
            '17': {'13':math.dist(self.map[17],self.map[13]),'16':math.dist(self.map[17],self.map[16]),'18':math.dist(self.map[17],self.map[18])},
            '18': {'17':math.dist(self.map[18],self.map[17]),'19':math.dist(self.map[18],self.map[19])},
            '19': {'9':math.dist(self.map[19],self.map[9]),'18':math.dist(self.map[19],self.map[18])},

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

    def complete_road(self):
        #initialization of the final road
        A = self.road[0]
        self.completed_road.append(A)
        B = self.road[1]
        # 20m = max distance for a turn
        dist_newpoint_B = round(17-0.1 * self.angles[0])
        while dist_newpoint_B >= math.dist(A, B):
            dist_newpoint_B -= 0.5
        newpoint = self.computation_position_newpoint(A, B, dist_newpoint_B)
        self.completed_road.append(newpoint)
        self.completed_road.append(B)  # we will need the corner point later even if it's not in the trajectory
        self.type_equation.append(0)

        A = self.road[2]
        while dist_newpoint_B >= math.dist(A,B):
            dist_newpoint_B -= 0.5
        newpoint = self.computation_position_newpoint(A, B, dist_newpoint_B)
        self.completed_road.append(newpoint)
        self.type_equation.append(1)

        for i in range(1, len(self.angles)):
            for j in range(2):
                if j == 0:
                    #entry in the corner
                    A = self.road[i]
                    B = self.road[i+1]
                    dist_newpoint_B = round(17 - 0.1 * self.angles[i], 2)
                    while dist_newpoint_B >= math.dist(A, B):
                        dist_newpoint_B -= 0.5

                if j == 1:
                    #exit of the corner
                    A = self.road[i+2]
                    B = self.road[i+1]
                    self.completed_road.append(B) #we will need the corner point later


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
        graph = self.visibility_graph()

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
            raise("The equation has no solution")
        sol = [list(x) for x in sol]
        solution = [0, 0]
        solution[0] = round(sol[0][0], 2)
        solution[1] = round(sol[1][0], 2)
        r = math.dist(solution, point1)  # sortir les valeurs calculées

        # Calcul angles de depart, d'arrivée, et angle de l'arc
        if (point1[0] - solution[0]) == 0:
            if (point1[1] - solution[1]) > 0:
                theta1 = math.pi / 2
            else:
                theta1 = -math.pi / 2
        else:
            theta1 = math.atan2((point1[1] - solution[1]), (point1[0] - solution[0]))

        if (point2[0] - solution[0]) == 0:
            if (point2[1] - solution[1]) > 0:
                theta2 = math.pi / 2
            else:
                theta2 = -math.pi / 2
        else:
            theta2 = math.atan2((point2[1] - solution[1]), (point2[0] - solution[0]))

        if theta1 == math.pi:
            if theta2 < 0:
                theta1 = -theta1

        if theta2 == math.pi:
            if theta1 < 0:
                theta2 = -theta2

        theta = theta2 - theta1
        if (abs(theta)>(2*math.pi-abs(theta))):
            if(theta1<0) and ((theta1+theta2)>0):
                theta = -(theta1 + theta2)
            else:
                theta=theta1+theta2

        for j in range(1, 11):  # 10 points par virage, à modifier
            t = j * theta / 10 + theta1

            x = round(solution[0] + r * math.cos(t), 2)
            y = round(solution[1] + r * math.sin(t), 2)

            point = []
            point.append(x)
            point.append(y)
            cornerlist.append(point)

        return cornerlist

    def final_list_calculator(self):
        final_list = []
        final_list.append(self.completed_road[0])
        counter = 0
        for i in range(len(self.type_equation)):
            distance = 1
            if self.type_equation[i] == 0:
                while distance <= math.dist(self.completed_road[counter], self.completed_road[counter + 1]):
                    newpoint = self.computation_position_newpoint(self.completed_road[counter + 1],
                                                                  self.completed_road[counter],
                                                                  distance)  # inversed because we want the distance from self.completed_road[counter+1] to decrease
                    distance += 1
                    final_list.append(newpoint)

            elif self.type_equation[i] == 1:
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



def point_on_map(map):
    x=map[:,0]
    y=map[:,1]
    img = plt.imread("googlemapp.png")
    fig, ax = plt.subplots()
    ax.imshow(img)
    plt.gca().invert_yaxis()
    ax.plot(x, y, '.', linewidth=5, color='firebrick')
    plt.show()

