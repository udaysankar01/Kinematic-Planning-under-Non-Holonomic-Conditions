import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

class World:

    def __init__(self, robotType):
        """
        This class defines all the assets in the simulated world and helps in visualizing it.
        """
        self.robotType = robotType

        if robotType == 1 or robotType == 2:
            # center obstacle
            self.obstacle_x = 75
            self.obstacle_y = 70
            self.obstacle_width = 60
            self.obstacle_height = 60

            self.coll_layer = 5    # collision layer     
            self.obstacle = [[self.obstacle_x - self.coll_layer, self.obstacle_y - self.coll_layer],
                            [self.obstacle_x - self.coll_layer, self.obstacle_y + self.obstacle_height +self.coll_layer],
                            [self.obstacle_x + self.obstacle_width - self.coll_layer, self.obstacle_y +self.coll_layer],
                            [self.obstacle_x + self.obstacle_width - self.coll_layer, self.obstacle_y + self.obstacle_height +self.coll_layer]]

            # vehicles
            self.vehicle1_x = 5
            self.vehicle1_y = 5
            self.vehicle2_x = 125
            self.vehicle2_y = 5
            self.vehicle_width = 45
            self.vehicle_height = 20

            self.vehicle1 = [[self.vehicle1_x - self.coll_layer, self.vehicle1_y - self.coll_layer],
                            [self.vehicle1_x - self.coll_layer, self.vehicle1_y + self.vehicle_height +self.coll_layer],
                            [self.vehicle1_x + self.vehicle_width - self.coll_layer, self.vehicle1_y +self.coll_layer],
                            [self.vehicle1_x + self.vehicle_width - self.coll_layer, self.vehicle1_y + self.vehicle_height +self.coll_layer]]

            self.vehicle2 = [[self.vehicle2_x - self.coll_layer, self.vehicle2_y - self.coll_layer],
                            [self.vehicle2_x - self.coll_layer, self.vehicle2_y + self.vehicle_height +self.coll_layer],
                            [self.vehicle2_x + self.vehicle_width - self.coll_layer, self.vehicle2_y +self.coll_layer],
                            [self.vehicle2_x + self.vehicle_width - self.coll_layer, self.vehicle2_y + self.vehicle_height +self.coll_layer]]

            # robot
            self.robot_x = 5
            self.robot_y = 135
            self.robot_theta = 0
            self.wheel_radius = 1
            self.start = [self.robot_x + 10, self.robot_y + self.vehicle_height / 2, 0]
            self.goal = [self.vehicle1_x + self.vehicle_width + 30, 10 + self.vehicle_height / 2, 0]
            self.parkingFlag = False
            self.velocity_tyre = [5,2,1,0,-1,-2,-5]

            self.wheel_base = 10

            self.robot_boundary = [[self.robot_x, self.robot_y, 1],
                            [self.robot_x + self.vehicle_width, self.robot_y, 1],
                            [self.robot_x, self.robot_y + self.vehicle_height, 1],
                            [self.robot_x + self.vehicle_width, self.robot_y + self.vehicle_height, 1]]
            
            
            
            self.robot_bound = [[-10,10,10,-10],[-10,-10,10,10],[1,1,1,1]]

            if robotType == 2:

                self.wheel_base = 28
                self.velocity = 1
                self.steering_angle = 50


                self.robot_bound = [[-1, 29, 9, -10], [-10, -10, 10, 10], [1, 1, 1, 1]]


        if robotType == 3:

            # center obstacle
            self.obstacle_x = 75
            self.obstacle_y = 70 
            self.obstacle_width = 50 
            self.obstacle_height = 50 

            self.coll_layer = 2     # collision layer     
            self.obstacle = [[self.obstacle_x - self.coll_layer, self.obstacle_y - self.coll_layer],
                             [self.obstacle_x + self.obstacle_width + self.coll_layer, self.obstacle_y - self.coll_layer],
                             [self.obstacle_x + self.obstacle_width + self.coll_layer, self.obstacle_y + self.obstacle_height + self.coll_layer],
                             [self.obstacle_x - self.coll_layer, self.obstacle_y + self.obstacle_height + self.coll_layer]]

            # vehicles
            self.vehicle1_x = 30 
            self.vehicle1_y = 10
            self.vehicle_width = 22
            self.vehicle_height = 17.5

            self.vehicle1 = [[self.vehicle1_x - self.coll_layer, self.vehicle1_y - self.coll_layer],
                             [self.vehicle1_x + self.vehicle_width + self.coll_layer, self.vehicle1_y - self.coll_layer],
                             [self.vehicle1_x + self.vehicle_width + self.coll_layer, self.vehicle1_y + self.vehicle_height + self.coll_layer],
                             [self.vehicle1_x - self.coll_layer, self.vehicle1_y + self.vehicle_height + self.coll_layer]]


            # robot
            self.robot_x = 49 # 5
            self.robot_y = 180 # 135
            self.robot_theta = 0
            self.wheel_radius = 1
            self.start = [self.robot_x, self.robot_y + self.vehicle_height / 2, 0]
            self.goal = [self.vehicle1_x + 2 * self.vehicle_width + 30, 10 + self.vehicle_height / 2, 180]
            self.parkingFlag = False
            self.velocity_tyre = [5,2,1,0,-1,-2,-5]

            self.wheel_base = 15
            self.wheel_radius = 1    
            self.steering_angle = 40
            self.hitching_length = 25
            self.velocity = 1
            self.velocity_tyre = [5,2,1,0,-1,-2,-5]
            self.parkingFlag = False

            self.trailer_x = self.robot_x - 23
            self.trailer_y = self.robot_y
            self.trailer_theta = 0
            self.start_trailer = [self.trailer_x,self.trailer_y +self.vehicle_height/2,0]

            self.robot_boundary = [[self.robot_x,self.robot_y,1],
                                   [self.robot_x+self.vehicle_width,self.robot_y,1],
                                   [self.robot_x+self.vehicle_width,self.robot_y+self.vehicle_height,1],
                                   [self.robot_x,self.robot_y+self.vehicle_height,1]]
            
            self.robot_bound = [[-1, 21, 21, -1],[-7.5, -7.5, 7.5, 7.5],[1, 1, 1, 1]]
            self.trailer_bound = [[-1, 11, 11, -1],[-7.5, -7.5, 7.5, 7.5],[1, 1, 1, 1]]

            
  
    def visualizeWorld(self, x, y, theta):
        """
        Helps in visualizing the world by including all the assets into a Matplotlib plot.
        """

        fig = plt.figure('robot world')
        ax = fig.add_subplot(111)
        
        
        if self.robotType == 1 or self.robotType == 2:
            plt.xlim([0, 200])
            plt.ylim([-10, 160])

            # define the assets
            draw_obstacle = Rectangle((self.obstacle_x, self.obstacle_y), self.obstacle_width, self.obstacle_height, color= "black")
            draw_vehicle1 = Rectangle((self.vehicle1_x, self.vehicle1_y), self.vehicle_width, self.vehicle_height, color= "red")
            draw_vehicle2 = Rectangle((self.vehicle2_x, self.vehicle2_y), self.vehicle_width, self.vehicle_height, color= "red")
            draw_start = Rectangle((self.robot_x, self.robot_y), self.vehicle_width, self.vehicle_height, color='green')
            draw_goal = Rectangle((self.vehicle1_x + self.vehicle_width + 15, 5), self.vehicle_width, self.vehicle_height, fc='#90EE90', ec='gray', lw=2)

            robot_boundary = self.get_robot_boundary(x, y, theta)
            robot_x, robot_y = robot_boundary[0]
            draw_robot = Rectangle((robot_x, robot_y-5), 20, 20, angle = theta, fc='red', lw=2)

            # add the assets in the world
            ax.add_patch(draw_obstacle)
            ax.add_patch(draw_vehicle1)
            ax.add_patch(draw_vehicle2) 
            ax.add_patch(draw_start)
            ax.add_patch(draw_goal)
            ax.add_patch(draw_robot)

            # Convert the matplotlib plot to an OpenCV image
            fig.canvas.draw()
            img_data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            img_data = img_data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

            # # Convert the image from RGB to BGR format, as OpenCV uses BGR by default
            # img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)

            # Display the image using OpenCV
            cv2.imshow('Animation', img_data)
            cv2.waitKey(1)

            return img_data
        
        else:

            plt.xlim([0, 200])
            plt.ylim([-10, 200])

            # define the assets
            draw_obstacle = Rectangle((self.obstacle_x-15, self.obstacle_y), self.obstacle_width, self.obstacle_height, color= "black")
            draw_vehicle1 = Rectangle((self.vehicle1_x, self.vehicle1_y), self.vehicle_width, self.vehicle_height, color= "red")
            draw_start = Rectangle((self.trailer_x-20, self.trailer_y-10), 2*self.vehicle_width, 1.5*self.vehicle_height, color='green')
            draw_goal = Rectangle((self.vehicle1_x + 2*self.vehicle_width, 5), 2*self.vehicle_width, 1.75*self.vehicle_height, fc='#90EE90', ec='gray', lw=2)
            robot_boundary, trailer_boundary = self.get_robot_boundary(x, y, theta)
            robot_x, robot_y = robot_boundary[0]
            draw_robot = Rectangle((robot_x-15, robot_y-5), 20, 20, angle = theta[0], fc='red', lw=2)

            trailer_x, trailer_y = trailer_boundary[0]
            draw_trailer= Rectangle((trailer_x-15, trailer_y), 20, 20, angle = theta[1], fc='red', lw=2)

            ax.add_patch(draw_start)
            ax.add_patch(draw_trailer)
            ax.add_patch(draw_vehicle1)
            ax.add_patch(draw_obstacle)
            ax.add_patch(draw_goal)
            ax.add_patch(draw_robot)
            ax.add_patch(draw_trailer)

            # Convert the matplotlib plot to an OpenCV image
            fig.canvas.draw()
            img_data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
            img_data = img_data.reshape(fig.canvas.get_width_height()[::-1] + (3,))

            # # Convert the image from RGB to BGR format, as OpenCV uses BGR by default
            # img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)

            # Display the image using OpenCV
            cv2.imshow('Animation', img_data)
            cv2.waitKey(1)

            return img_data



    def get_robot_boundary(self, x, y, theta):
        """
        Computes the boundary information of the robot and if needed, the trailer attached to the truck case.
        """

        if self.robotType == 3:
            x_robot, x_trailer = x
            y_robot, y_trailer = y
            theta_robot, theta_trailer = theta


            theta_trailer_new = (theta_trailer - self.start_trailer[2]) * (math.pi / 180)
            theta_robot_new = (theta_robot- self.start[2]) * (math.pi / 180)

        else:
            theta_robot_new = (theta - self.start[2]) * (math.pi / 180)
            x_robot = x
            y_robot = y


        homo_matrix_robot = [[math.cos(theta_robot_new), -math.sin(theta_robot_new), x_robot],
                       [math.sin(theta_robot_new), math.cos(theta_robot_new), y_robot]]
        
        robot_matrix = np.dot(homo_matrix_robot, self.robot_bound)
        robot_updated_boundary = [[robot_matrix[0][0], robot_matrix[1][0]],
                            [robot_matrix[0][1], robot_matrix[1][1]],
                            [robot_matrix[0][2], robot_matrix[1][2]],
                            [robot_matrix[0][3], robot_matrix[1][3]]]
        
        if self.robotType == 3:
            homo_matrix_trailer = [[math.cos(theta_trailer_new), -math.sin(theta_trailer_new), x_trailer],
                       [math.sin(theta_trailer_new), math.cos(theta_trailer_new), y_trailer]]
            
            trailer_matrix = np.dot(homo_matrix_trailer, self.trailer_bound)
            trailer_updated_boundary = [[trailer_matrix[0][0], trailer_matrix[1][0]],
                                [trailer_matrix[0][1], trailer_matrix[1][1]],
                                [trailer_matrix[0][2], trailer_matrix[1][2]],
                                [trailer_matrix[0][3], trailer_matrix[1][3]]]
            

            return robot_updated_boundary, trailer_updated_boundary
        
        return robot_updated_boundary