import math
from queue import Queue

class Planner:

    def __init__(self, world, robotType):
        """
        This is the path planner class which responsible for planning a path for the given robot type.
        """

        self.robotType = robotType
        self.world = world

        self.start = world.start
        self.goal = world.goal
        self.velocity_tyre = world.velocity_tyre
        self.wheel_radius = world.wheel_radius
        self.wheel_base = world.wheel_base
        self.vehicle_height = world.vehicle_height
        self.vehicle_width = world.vehicle_width
        self.obstacle = world.obstacle
        self.vehicle1 = world.vehicle1

        if self.robotType != 3:
            self.vehicle2 = world.vehicle2
        self.park_point = world.parkingFlag

        if robotType == 2 or robotType == 3:
            self.velocity = world.velocity
            self.steering_angle = world.steering_angle
            
            if robotType == 3:
                self.start_trailer = world.start_trailer
                self.hitching_length = world.hitching_length

    def getNeighbourNodes(self,x,y,theta):

        neighbour_nodes = []

        if self.robotType == 1:
            
            for velocity_right in self.velocity_tyre:
                for velocity_left in self.velocity_tyre:
                    if velocity_left == 0 and velocity_right == 0:
                        continue

                    vel = (velocity_left + velocity_right) / 2
                    x_dot = vel * math.cos(theta * (math.pi / 180))
                    y_dot = vel * math.sin(theta * (math.pi / 180))
                    theta_dot = (self.wheel_radius / (2 * self.wheel_base)) * (velocity_left - velocity_right) * (180 / math.pi)

                    if(self.checkValidNode(x + x_dot, y + y_dot, theta + theta_dot)):
                        neighbour_nodes.append([round(x + x_dot, 2), round(y + y_dot, 2), (round(theta + theta_dot, 2)) % 360, velocity_left, velocity_right])

        elif self.robotType == 2:

            for i in range(-self.steering_angle, self.steering_angle + 1, 5):

                x_dot = self.velocity * math.cos(theta * (math.pi / 180))
                y_dot = self.velocity * math.sin(theta * (math.pi / 180))
                theta_dot = (self.velocity * math.tan(i *(math.pi / 180)) / self.wheel_base) * (180 / math.pi)

                if(self.checkValidNode(x + x_dot, y + y_dot, theta + theta_dot)):
                    neighbour_nodes.append([round(x + x_dot, 2), round(y + y_dot, 2), (round(theta + theta_dot, 2)) % 360, 1, i])
                if(self.checkValidNode(x-x_dot,y-y_dot,theta-theta_dot)):
                    neighbour_nodes.append([round(x - x_dot, 2), round(y - y_dot, 2), (round(theta - theta_dot, 2)) % 360, -1, i])

        elif self.robotType == 3:

            for i in [-self.steering_angle,0,self.steering_angle] :

                x_robot, x_trailer = x
                y_robot, y_trailer = y
                theta_robot, theta_trailer = theta

                x_robot_dot = self.velocity * math.cos(theta_robot * (math.pi / 180))
                y_robot_dot = self.velocity * math.sin(theta_robot * (math.pi / 180))
                theta_robot_dot = (self.velocity * math.tan(i * (math.pi / 180)) / self.wheel_base) * (180 / math.pi)

                x_trailer_dot = self.velocity * math.cos(theta_trailer * (math.pi / 180))
                y_trailer_dot = self.velocity * math.sin(theta_trailer * (math.pi / 180))
                theta_trailer_dot = (self.velocity*math.sin((theta_robot-theta_trailer) * (math.pi / 180)) / self.hitching_length) * (180 / math.pi)

                if(self.checkValidNode([x_robot + x_robot_dot, x_trailer + x_trailer_dot], [y_robot+y_robot_dot, y_trailer+y_trailer_dot], [theta_robot+theta_robot_dot, theta_trailer+theta_trailer_dot])):
                    neighbour_nodes.append([round(x_robot+x_robot_dot,2),round(y_robot+y_robot_dot,2),(round(theta_robot+theta_robot_dot,2))%360,round(x_trailer+x_trailer_dot,2),round(y_trailer+y_trailer_dot,2),(round(theta_trailer+theta_trailer_dot,2)+360)%360,1,i])
                if(self.checkValidNode([x_robot-x_robot_dot, x_trailer-x_trailer_dot], [y_robot-y_robot_dot, y_trailer-y_trailer_dot], [theta_robot-theta_robot_dot, theta_trailer-theta_trailer_dot])):
                    neighbour_nodes.append([round(x_robot-x_robot_dot,2),round(y_robot-y_robot_dot,2),(round(theta_robot-theta_robot_dot,2))%360,round(x_trailer-x_trailer_dot,2),round(y_trailer-y_trailer_dot,2),(round(theta_trailer-theta_trailer_dot,2)+360)%360,-1,i])

        return neighbour_nodes


    def checkValidNode(self, x, y, theta):
        
        if self.robotType != 3:
            robot_boundary = self.world.get_robot_boundary(x, y, theta)
            if x < 1 or y < self.vehicle_height or x > 200 - self.vehicle_width or y > 200 - self.vehicle_height / 2.0:
                return False
            
            collision_condition1 = self.checkPolygonIntersection(robot_boundary, self.obstacle)
            collision_condition2 = self.checkPolygonIntersection(robot_boundary, self.vehicle1)
            collision_condition3 = self.checkPolygonIntersection(robot_boundary, self.vehicle2)

            if (collision_condition1 or collision_condition2 or collision_condition3):
                return False
            
        else:
            x_robot, x_trailer = x
            y_robot, y_trailer = y
            theta_robot, theta_trailer = theta

            boundary_robot, boundary_trailer = self.world.get_robot_boundary([x_robot, x_trailer], [y_robot, y_trailer], [theta_robot, theta_trailer])
            if x_robot < 1 or y_robot < self.vehicle_height or x_robot > 200 - self.vehicle_width or y_robot > 200 - self.vehicle_height / 2.0:
                return False
            collision_condition1 = self.checkPolygonIntersection(boundary_robot,self.obstacle)
            collision_condition2 = self.checkPolygonIntersection(boundary_robot,self.vehicle1)
            collision_condition3 = self.checkPolygonIntersection(boundary_trailer,self.vehicle1)
            collision_condition4 = self.checkPolygonIntersection(boundary_trailer,self.obstacle)
            if (collision_condition1 or collision_condition2 or collision_condition3 or collision_condition4):
                return False
        
        return True


    def checkPolygonIntersection(self, polygon_a, polygon_b):
        polygons = [polygon_a, polygon_b]
        for i in range(len(polygons)):
            polygon = polygons[i]
            for j in range(len(polygon)):
                v1 = polygon[j]
                v2 = polygon[(j + 1) % len(polygon)]
                normal = {'x': v2[1] - v1[1], 'y': v1[0] - v2[0]}
                min_a, max_a, min_b, max_b = None, None, None, None
                for k in range(len(polygon_a)):
                    projected = normal['x'] * polygon_a[k][0] + normal['y'] * polygon_a[k][1]
                    if min_a is None or projected < min_a:
                        min_a = projected
                    if max_a is None or projected > max_a:
                        max_a = projected
                for k in range(len(polygon_b)):
                    projected = normal['x'] * polygon_b[k][0] + normal['y'] * polygon_b[k][1]
                    if min_b is None or projected < min_b:
                        min_b = projected
                    if max_b is None or projected > max_b:
                        max_b = projected
                if max_a < min_b or max_b < min_a:
                    return False
        return True

      

    def compute_cost(self, x1, y1, x2, y2):
        return math.sqrt((pow(x1 - x2, 2) + pow(y1 - y2, 2)))


    def compute_heuristic_diff(self, x, y, theta, velocity_left, velocity_right):

        theta_hat = 0
        theta = (theta + 360) % 360
        vel = (velocity_left + velocity_right) / 2
        reverse_penalty = 0
        parking_penalty = 0
        distance = math.sqrt((pow(self.goal[0] - x, 2) + pow(self.goal[1] - y, 2)))

        if self.checkStraightAvailable(x, y) and not (self.goal[0] - 1 <= x <= self.goal[0] + 1 and self.goal[1] - 1 <= y <= self.goal[1] + 1):
            theta_hat = abs((360 + (math.atan2(y - self.goal[1], x - self.goal[0])) * (180 / math.pi)) % 360 - theta + 180)
        else:
            theta_hat = (self.goal[2] - theta + 360) % 360
        if vel < 0:
            reverse_penalty = 1
        
        if (x > self.goal[0]-1 and y > self.goal[1] - 1 and x < self.goal[0] + 1 and y < self.goal[1] + 1):
            self.park_point = True
        
        if self.park_point and not (self.goal[0] - 1 <= x <= self.goal[0] + 1 and self.goal[1] - 1 <= y <= self.goal[1] + 1) and self.velocity_tyre == self.velocity_tyre:
            parking_penalty = 180

        return distance + theta_hat + reverse_penalty + parking_penalty
        
    
    def compute_heuristic_ackermann(self, x, y, theta):

        theta_hat = 0
        theta = (theta + 360) % 360

        distance_to_goal = math.sqrt((pow(self.goal[0] - x, 2) + pow(self.goal[1] - y, 2)))
        distance_to_goal += math.sqrt(((pow((self.goal[0] + self.vehicle_width) - (x + self.vehicle_width * math.cos(theta * (math.pi / 180))), 2) + 
                                pow((self.goal[1] + self.vehicle_height) - (y + self.vehicle_height * math.sin(theta * (math.pi / 180))), 2))))
        
        if self.checkStraightAvailable(x, y) and not(x > self.goal[0] - 1 and y > self.goal[1] - 1 and x < self.goal[0] + 1 and y < self.goal[1] + 1):
            theta_hat = abs((360 + (math.atan2(y - self.goal[1], x - self.goal[0])) * (180 / math.pi)) % 360 - theta + 180)

        return distance_to_goal + theta_hat 
    
    
    def compute_heuristic_trailer(self, x, y, theta, vel):

        x_robot, x_trailer = x
        y_robot, y_trailer = y
        theta_robot, theta_trailer = theta

        estimated_theta = 0
        theta_robot = (theta_robot + 360) % 360
        theta_trailer = (theta_trailer + 360) % 360

        if theta_trailer == 0:
            theta_trailer= 360

        reverse_penalty = 0
        turning_penalty  = 0
        obstacle_penalty = 0
        distance = math.sqrt((pow(self.goal[0] - x_robot, 2) + pow(self.goal[1] - y_robot, 2)))

        distance += math.sqrt(((pow((self.goal[0] - self.vehicle_width) - (x_robot + self.vehicle_width * math.cos(theta_robot * (math.pi / 180))), 2) + pow((self.goal[1] + self.vehicle_height) - (y_robot + self.vehicle_height * math.sin(theta_robot * (math.pi / 180))), 2))))
        
        if self.checkStraightAvailable([x_robot, x_trailer], [y_robot, y_trailer]) and not(x_robot > self.goal[0] - 5 and y_robot > self.goal[1] - 5 and x_robot < self.goal[0] + 5 and y_robot < self.goal[1] + 5):
            estimated_theta = abs((360 + (math.atan2(y_robot - self.goal[1], x_robot - self.goal[0])) * (180 / math.pi)) % 360 - theta_robot + 180)
        else:
            estimated_theta = 180
        if self.checkPolygonIntersection([[x_robot - 15, y_robot],[x_robot + 200 * math.cos(theta_robot * (math.pi / 180)) - 15, y_robot + 200 * math.sin(theta_robot * (math.pi / 180))], [15 + x_robot + 200 * math.cos(theta_robot * (math.pi / 180)), y_robot + 200 * math.sin(theta_robot * (math.pi / 180))], [x_robot + 15, y_robot]], self.obstacle):
            obstacle_penalty +=10
        if self.checkPolygonIntersection([[x_robot - 15, y_robot],[x_robot + 200 * math.cos(theta_robot * (math.pi / 180)) - 15, y_robot + 200 * math.sin(theta_robot * (math.pi / 180))], [15 + x_robot + 200 * math.cos(theta_robot * (math.pi / 180)), y_robot + 200 * math.sin(theta_robot * (math.pi / 180))], [x_robot + 15, y_robot]], self.obstacle):
            obstacle_penalty +=10  
        if vel < 0:
            reverse_penalty = 1
        if abs(theta_robot - theta_trailer) > 15 and not(x_robot > self.goal[0] - 5 and y_robot > self.goal[1] - 5 and x_robot < self.goal[0] + 5 and y_robot < self.goal[1] + 5):
            turning_penalty  = 5
        
        return distance + estimated_theta + reverse_penalty + turning_penalty  + obstacle_penalty 


    
    def checkStraightAvailable(self,x,y):
        
        if self.robotType != 3:

            boundary_line = [[x, y], [self.goal[0], self.goal[1]], [self.goal[0] + 1, self.goal[1]], [x + 1, y]]
            if (self.checkPolygonIntersection(boundary_line, self.obstacle)) or (self.checkPolygonIntersection(boundary_line, self.vehicle1)):
                return False
        
        else:

            x_robot, x_trailer = x
            y_robot, y_trailer = y

            boundary_line_robot = [[x_robot, y_robot], [self.goal[0], self.goal[1]], [self.goal[0] + 1, self.goal[1]], [x_robot + 1, y_robot]]
            boundary_line_trailer = [[x_trailer,y_trailer], [self.goal[0] + 50, self.goal[1]], [self.goal[0] + 1 + 50, self.goal[1]], [x_trailer + 1, y_robot]]
            if (self.checkPolygonIntersection(boundary_line_robot, self.obstacle)) or (self.checkPolygonIntersection(boundary_line_robot, self.vehicle1)) or (self.checkPolygonIntersection(boundary_line_trailer, self.obstacle)) or (self.checkPolygonIntersection(boundary_line_trailer, self.vehicle1)):
                return False
        return True



    def findPriority(self, queue):

        min = math.inf
        index = 0

        for i in range(len(queue)):
            value = queue[i][1]
            if value < min:
                min = value
                index = i

        return index
    

    def checkVisited(self, curr_node, visited):

        if self.robotType != 3:
            for x, y, theta in visited:
                if curr_node[0] == x and curr_node[1] == y and curr_node[2] == theta:
                    return True
        else:
            for x, y, theta, x_trailer, y_trailer, theta_trailer in visited:
                if curr_node[0] == x and curr_node[1] == y and curr_node[2] == theta and curr_node[3] == x_trailer and curr_node[4] == y_trailer and curr_node[5] == theta_trailer:
                    return True
            
        return False


    def A_star(self):
        """
        This method performs A* path planning algorithm on the given robot type.
        """
        queue = []
        visited = []
        f = 0
        g = 0

        if self.robotType != 3:
            start = self.start
        else:
            start = [self.start[0],self.start[1],self.start[2],self.start_trailer[0],self.start_trailer[1],self.start_trailer[2]]

        path = [start]
        queue.append((start, f, g, path))


        if self.robotType == 1:

            while len(queue) > 0:
                index = self.findPriority(queue)
                (shortest,_,g_value,path) = queue[index]
                queue.pop(index)
                if not (self.checkVisited((round(shortest[0]),round(shortest[1]),round(shortest[2])),visited)):
                    visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2])])
                    if shortest[2] > 180:
                        angle = 360 - shortest[2]
                    else:
                        angle = shortest[2]
                    
                    if round(shortest[0]) <= self.goal[0]+1 and round(shortest[0]) >= self.goal[0]-1 and round(shortest[1]) <= self.goal[1]+1 and round(shortest[1]) >= self.goal[1]-1 and angle <= self.goal[2]+5 and angle >= self.goal[2]-5:
                        return path
                    neighbours = self.getNeighbourNodes(shortest[0],shortest[1],shortest[2])
                    for neighbour in neighbours:
                        vl = neighbour[3]
                        vr = neighbour[4]
                        t_g = g_value + (0.1*self.compute_cost(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                        t_f = t_g +(0.9*self.compute_heuristic_diff(neighbour[0],neighbour[1],neighbour[2],vl,vr))
                        queue.append((neighbour,t_f,t_g,path + [neighbour]))
        

        elif self.robotType == 2:

            while len(queue) > 0:
                index = self.findPriority(queue)
                (shortest,_,g_value,path) = queue[index]
                queue.pop(index)
                if not (self.checkVisited((round(shortest[0]),round(shortest[1]),round(shortest[2])),visited)):
                    visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2])])
                    
                    if round(shortest[0]) <= self.goal[0]+5 and round(shortest[0]) >= self.goal[0]-5 and round(shortest[1]) <= self.goal[1]+5 and round(shortest[1]) >= self.goal[1]-5 and shortest[2] <= self.goal[2]+15 and shortest[2] >= self.goal[2]-15:
                        return path
                    neighbours = self.getNeighbourNodes(shortest[0],shortest[1],shortest[2])
                    for neighbour in neighbours:
                        t_g = g_value + (0.1*self.compute_cost(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                        t_f = t_g +(0.9*self.compute_heuristic_ackermann(neighbour[0],neighbour[1],neighbour[2]))
                        queue.append((neighbour,t_f,t_g,path + [neighbour]))
        
    
        else:

            while len(queue) > 0:
                index = self.findPriority(queue)
                (shortest,_,g_value,path) = queue[index]
                queue.pop(index)
                if not (self.checkVisited([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])],visited)):
                    visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])])
                    if round(shortest[0]) <= self.goal[0]+5 and round(shortest[0]) >= self.goal[0]-5 and round(shortest[1]) <= self.goal[1]+5 and round(shortest[1]) >= self.goal[1]-5 and shortest[2] <= self.goal[2]+15 and shortest[2] >= self.goal[2]-15:
                        return path
                    neighbours = self.getNeighbourNodes([shortest[0],shortest[3]], [shortest[1],shortest[4]], [shortest[2],shortest[5]])
                    for neighbour in neighbours:
                        vel = neighbour[6]
                        t_g = g_value + (0.1*self.compute_cost(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                        t_f = t_g +(0.9*self.compute_heuristic_trailer([neighbour[0],neighbour[3]], [neighbour[1],neighbour[4]], [neighbour[2],neighbour[5]], vel))
                        queue.append((neighbour,t_f,t_g,path + [neighbour]))

        return path
    
        
    