from src.robot import edward_bot
from src.Map import Map_plot as maps
import queue
import numpy as np
import sys
import os

class search:
    def __init__(self,start_position,goal_position, goal_distance, map_supplied : maps,robot_supplied : edward_bot,s_no,initial_theta = 0):
        '''
        This Class implements a search algorithm. Currently supports only A* algorithm.
        :param start_position: The start position of the robot
        :param goal_position: The goal position of the robot
        :param goal_distance: The distance where the search algorithm has to stop
        :param map_supplied: The Map object createf for the environment
        :param robot_supplied: The Robot object created for the Robot
        :param s_no: The serial number of the robot. Starting from 1
        :param initial_theta: The initial theta (orientation of the robot in z-direction) of the Robot
        '''

        self.robot = robot_supplied
        self.start_position = start_position
        self.goal_position = goal_position
        self.initial_theta = initial_theta
        self.serial = s_no
        self.goal_distance = goal_distance

        self.map = map_supplied

        self.nodes_list = []
        self.nodes_list.append([self.start_position])
        self.start_position_theta = [self.start_position[0],self.start_position[1],self.initial_theta]

        self.q = queue.PriorityQueue()
        self.q.put([0, self.start_position_theta])

        self.node_check_set = set([])            #visited nodes
        # self.node_check_set.add(str(self.start_position))
        self.node_check_set.add(str(self.start_position_theta))

        self.node_info_dict = self.node_info_list(self.map.x_min,self.map.y_min,self.map.x_max,self.map.y_max)
        self.node_info_dict[str([self.start_position])] = 0

        self.node_info_parent_dict = {}
        # self.node_info_velocities = {}

        self.visited_node = []
        self.is_a_vaid_input = self.valid_start_goal()
        self.goal_reached = False
        self.goal_obtained = []

        # self.current_node = self.start_position

    def is_visited_check(self,node):
        '''
        Check whether a node has been already visited
        :param node: Current node which has to be checked
        :return: True or False
        '''
        return str(node) in self.node_check_set


    def node_info_list(self,min_x,min_y,max_x,max_y):
        '''
        Creates all the nodes along with infinite cost as a dictionary
        :param min_x: minimum x-value
        :param min_y: minimum y-value
        :param max_x: maximum x-value
        :param max_y: maximum y-value
        :return: Returns the dictionary with Nodes and their initial costs
        '''
        loc = []
        for i in range(int(min_x),int(max_x),1):
            for j in range(int(min_y),int(max_y),1):
                loc.append([i,j])
        distance = {}
        for node in loc:
            distance[str(node)] = 9999999
        return distance


    def valid_start_goal(self):
        '''
        Check if the start position and goal positions are Valid
        :return: Returns True or False
        '''

        # status = True

        if (self.map.check_obs(self.start_position) or self.map.check_obs(self.goal_position)):
            # status = False
            sys.exit('either the start_position or your goal position is in obstacle space, enter a valid input')

        else:
            status = True

        return status


    def cost_to_go(self,current_node):
        '''
        Computes the cost to go to goal from the current position
        :param current_node: The current node
        :return:
        '''

        x1 = current_node[0]
        y1 = current_node[1]

        x2 = self.goal_position[0]
        y2 = self.goal_position[1]

        d = np.sqrt((x2-x1)**2 + (y2-y1)**2)

        return d

    def is_near_goal(self, node):
        '''
        Computes if the current node is near to goal
        :param node: Current position of the node
        :return: True or False
        '''
        # ( "distance between self.goal_position and node[1] is less than self.goal_distance")
        x1 = node[0]
        y1 = node[1]

        x2 = self.goal_position[0]
        y2 = self.goal_position[1]

        dist = np.sqrt((x1-x2)**2 + (y1-y2)**2)

        print('is',dist,' cm far from the goal')

        if dist <= self.goal_distance:
            print('less than min goal_to_distance ')
            return True
        else:
            return False

    def A_star(self):
        '''
        Implements the A-Star search Algorithm
        :return: Returns the path from start position to goal position
        '''

        iter1 = 0
        self.node_path_temp = []

        while not self.q.empty() and self.is_a_vaid_input == True:# and :

            node = self.q.get()      ##[0, [15, 20, 0]] --> [cost, [x,y,theta]]
            print('node in while',node)

            x_new = node[1][0]
            y_new = node[1][1]
            theta_new = node[1][2]
            node_pos = [node[1][0],node[1][1]]

            self.node_path_temp.append(node_pos)

            iter1+=1
            # print(iter1)

            # if ([int(node[1][0]),int(node[1][1])] == self.goal_position):

            if ([int(node[1][0]),int(node[1][1])] == self.goal_position) or self.is_near_goal(node[1]):
                print('goal reached',iter1)
                self.goal_obtained = [node[1][0],node[1][1]]
                self.goal_reached = True
                break
            explored_nodes = self.robot.next_action_set(x_new,y_new,theta_new)
            # print(iter1, explored_nodes)

            # action = np.array([new_x,new_y,new_theta,new_cost,action[0],action[1]])
            for action in explored_nodes:

                # print(action)  ##[1.98308572 3.1073315  1.72727273 0.19       0.         5.        ]
                action_pos = [action[0],action[1]]
                action_theta = action[2]
                action_cost = action[3]

                # print("action_pos = ",action_pos, "action_theta", action_theta, "action_cost", action_cost)
                # action_velocities = [action[4],action[5],action_cost]

                # if self.is_visited_check(action_pos) == False:
                #
                #     if self.map.check_obs(action_pos) == False:
                #         self.node_check_set.add(str([int(action_pos[0]),int(action_pos[1])])) ## marked as visited --> added to visited nodes ## str([d(act[0]),d(act[1])])
                #         self.visited_node.append(action_pos)
                #         # print(str([int(action_pos[0]),int(action_pos[1])]))
                #         cost = action_cost + self.node_info_dict[str([int(action_pos[0]),int(action_pos[1])])] + self.cost_to_go(action_pos)
                #         self.node_info_dict[str([int(action_pos[0]),int(action_pos[1])])] = cost
                #         self.q.put([cost,[action[0],action[1],action_theta]])
                #         self.node_info_parent_dict[str(action_pos)] = node_pos#--> parent is updated to the node info
                #         # displayArray =updateAndDisplay(displayArray,[int(action[0]/displayRes),int(action[1]/displayRes)],3)
                #         # self.node_info_velocities[str(action_pos)] = action_velocities
                #
                #
                # else:
                #     if self.map.check_obs(action_pos) == False:
                #         temp = action_cost + self.node_info_dict[str(node_pos)]
                #         if self.node_info_dict[str(action_pos)] > temp:
                #             self.node_info_dict[str(action_pos)] = temp + self.cost_to_go(action_pos)
                #             self.node_info_parent_dict[str(action_pos)] = node_pos           #--> parent is updated to the node info
                #             # self.node_info_velocities[str(action_pos)] = action_velocities

                if self.is_visited_check(action_pos) == False:

                    if self.map.check_obs(action_pos) == False:
                        self.node_check_set.add(str([action_pos[0], action_pos[1]])) ## marked as visited --> added to visited nodes ## str([d(act[0]),d(act[1])])
                        self.visited_node.append(action_pos)
                        cost = action_cost + self.node_info_dict[str([int(action_pos[0]),int(action_pos[1])])] + self.cost_to_go(action_pos)
                        self.node_info_dict[str([int(action_pos[0]),int(action_pos[1])])] = cost

                        self.q.put([cost,[action[0],action[1],action_theta]])
                        self.node_info_parent_dict[str(action_pos)] = node_pos #--> parent is updated to the node info

                else:
                    if self.map.check_obs(action_pos) == False:
                        temp = action_cost + self.node_info_dict[str(node_pos)]
                        if self.node_info_dict[str(action_pos)] > temp:
                            self.node_info_dict[str(action_pos)] = temp + self.cost_to_go(action_pos)

        print("out of the while")

        self.node_path = []

        if self.is_a_vaid_input == True:
            if self.goal_reached:

                self.node_path.append(self.goal_obtained)
                parent = self.node_info_parent_dict[str(self.goal_obtained)]

                while parent != self.start_position:

                    parent =  self.node_info_parent_dict[str(parent)]
                    self.node_path.append(parent)

                print("Out of this while too ")

                self.node_path.reverse()

            else:
                print('goal not reached')
        else:
            print('enter a valid input')

        node_path_arr = np.asarray(self.node_path)

        base_name = 'track_positions'
        suffix = '.txt'

        fname = os.path.join(base_name + str(self.serial) + suffix)

        # print(fname)

        with open(fname, 'w') as velocities_file:

            for i in node_path_arr:
                p = np.empty([1,2])
                p[0:,] = i
                np.savetxt(velocities_file,p,delimiter='\t')

        return self.node_path


