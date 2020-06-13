import math
import numpy as np
# from src.map import maps

# class robot:
#
#     def __init__(self, robot_radius, rpm1, rpm2, map_supplied :maps,resolution, length = 0.23, wheel_rad =0.076,dt = 1):
#
#         self.map = map_supplied
#         self.res = resolution
#         self.robot_radius = robot_radius
#         # self.clearance = clearance
#         self.rpm1 = rpm1
#         self.rpm2 = rpm2
#         self.len = length*self.res
#         self.rad = wheel_rad*self.res/2
#         self.dt = dt
#
#     def calculate_values(self,cur_x,cur_y,cur_theta,u_left,u_right,dt):
#         x = cur_x
#         y = cur_y
#         theta= cur_theta
#         l = self.len
#         r = self.rad
#         ul = u_left
#         ur = u_right
#
#         dx = (r/2)*(ul+ur)*math.cos(theta)*dt
#         dy = (r/2)*(ul+ur)*math.sin(theta)*dt
#         d_theta = (r/l)*(ur-ul)*dt
#
#         x_new = x + dx #* math.cos(theta) - dy* math.sin(theta)
#         y_new = y + dy #* math.sin(theta) + dy* math.cos(theta)
#         theta = theta + d_theta
#         euclidean_distante = np.sqrt((x-x_new)**2 + (y-y_new)**2)
#
#         return x_new, y_new, theta, euclidean_distante
#
#     def new_position_integration(self,current_x,current_y,current_theta,vel1,vel2):
#
#         cur_x = current_x
#         cur_y = current_y
#         cur_theta = current_theta
#         u_left = vel1
#         u_right = vel2
#         new_cost = 0
#         integration_step  = math.ceil(self.dt/0.01)
#         new_time_step = self.dt/integration_step
#
#         for i in range(int(integration_step)):
#             cur_x , cur_y, cur_theta, cur_cost= self.calculate_values(cur_x,cur_y,cur_theta,u_left,u_right,new_time_step)
#             new_cost = cur_cost + new_cost
#
#         return cur_x,cur_y,cur_theta,new_cost
#
#     def action_set(self,current_x,current_y,theta):
#         # current_theta  = node_info[node_number,5]
#         current_theta = theta
#         action_list = []
#         action_set = np.array([[0,self.rpm1],
#                                [self.rpm1,0],
#                                [self.rpm1,self.rpm1],
#                                [0,self.rpm2],
#                                [self.rpm2,0],
#                                [self.rpm2,self.rpm2],
#                                [self.rpm1,self.rpm2],
#                                [self.rpm2,self.rpm1]
#                                ])
#
#         for action in action_set:
#             new_x,new_y,new_theta,new_cost = self.new_position_integration(current_x,current_y,current_theta,action[0],action[1])
#             if new_x >=  self.map.x_min  and new_y >=  self.map.y_min and new_x <=  self.map.x_max and new_y <=  self.map.y_max:
#
#                 next_action = np.array([new_x,new_y,new_theta,new_cost,action[0],action[1]])
#                 action_list.append(next_action)
#         return action_list

class edward_bot:

    # def __init__(self, robot_radius, x_step, y_step, theta_step, resolution, map_supplied :maps):
    def __init__(self,x_step, y_step, theta_step, resolution):
        '''

        :param x_step: Step size in x-direction
        :param y_step: Step size in y-direction
        :param theta_step: Step size in the z-axis rotation in degrees
        :param resolution: resolution
        '''

        # self.map = map_supplied
        self.res = resolution
        # self.robot_radius = robot_radius

        self.x_step =  x_step
        self.y_step = y_step
        self.theta_step = np.radians(theta_step)
        # self.clearance = clearance

    def check_validity(self, action):
        '''
        Checks if a action is valid or not
        :param action: the action for which validity needs to be checked
        :return: returns a bool, if the action is valid or not
        '''

        validity = False
        new_x = action[0]
        new_y = action[1]

        if new_x >=  self.map.min_x  and new_y >=  self.map.min_y and new_x <=  self.map.max_x and new_y <=  self.map.max_y:
            validity = True

        else:
            validity = False

        return validity

    def calculate_cost(self, cur_x, cur_y, action):
        '''
        Calculate the cost in performing this action
        :param cur_x: Current x-coordinate
        :param cur_y: Current y-coordinate
        :param action: the action to which cost needs to be calculated
        :return: Returns the cost in float
        '''

        x_new = action[0]
        y_new = action[1]

        cost = np.sqrt((cur_x-x_new)**2 + (cur_y-y_new)**2)

        return cost

    def next_action_set(self,current_x,current_y,current_theta):
        '''
        Computes the next set of actions from the current state
        :param current_x: Current x-coordinate
        :param current_y: Current y-coordinate
        :param current_theta: Current z-axis orientation in degrees
        :return action_list: next valid actions
        '''

        current_theta = np.radians(current_theta)
        action_list = []

        left = [current_x + self.x_step * math.cos( current_theta + self.theta_step ) ,
                current_y + self.y_step * math.sin( current_theta + self.theta_step ) ,
                np.rad2deg(current_theta + self.theta_step),
                0
        ]

        right = [current_x + self.x_step * math.cos( current_theta - self.theta_step ) ,
                 current_y + self.y_step * math.sin( current_theta - self.theta_step ) ,
                 np.rad2deg(current_theta - self.theta_step),
                 0
        ]

        forward = [current_x + self.x_step * math.cos( current_theta ) ,
                   current_y + self.y_step * math.sin( current_theta ) ,
                   np.rad2deg(current_theta),
                   0
        ]

        reverse = [current_x - self.x_step * math.cos( current_theta ) ,
                   current_y - self.y_step * math.sin( current_theta ) ,
                   np.rad2deg(current_theta),
                   0
        ]

        self.action_set = [ left ,
                       right,
                       forward,
                       reverse
        ]

        for action in self.action_set:
            valid = self.check_validity(action)

            if valid:
                cost = self.calculate_cost(current_x,current_y, action)
                action[3] = cost
                action_list.append(action)

        return action_list

# E = edward_bot(5,5,30,5)
#
#
# next_nodes = E.next_action_set(25,25,30)
#
# print(next_nodes)
