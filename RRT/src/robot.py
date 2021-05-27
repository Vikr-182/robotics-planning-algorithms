import numpy as np
import pygame
from copy import deepcopy

class RobotNonHolonomic_differential_drive():
    def __init__(self, spawn, goal, heading = [1,0], num_children = 7, d=4):
        '''
        spawn: initial position or World.spawn
        goal: target position
        num_children: number of children to protrude from each node of the RRT
        '''

        self.base_l = [spawn[0] + d*heading[1], (spawn[1] - heading[0]*d)]
        self.base_r = [spawn[0] - d*heading[1], (spawn[1] + heading[0]*d)]

        self.pos = spawn
        self.goal = goal
        self.unexplored_points = [spawn+heading+self.base_l+self.base_r+[None]+[None]]
        self.num_children = num_children
        
        self.explored_paths = []
        self.right_wheel_paths = []
        self.left_wheel_paths = []

        self.reach_threshold = 5
        self.d = d
        self.robot_size = 2*d
        self.final_path = []

    def print_robot(self, gameDisplay):
        '''
        prints robot on to <gameDisplay>
        '''
        pygame.draw.circle(gameDisplay,(0,0,255),self.pos, (3,3))

    def print_paths(self, gameDisplay, type = 'center', done = False):
        if not done:
            if type == 'center':
                for path in self.explored_paths:
                    pygame.draw.lines(gameDisplay, (147, 205, 218), False, path)
            elif type == 'wheels':
                for path in self.right_wheel_paths:
                    pygame.draw.lines(gameDisplay, (255, 116, 0), False, path)
                for path in self.left_wheel_paths:
                    pygame.draw.lines(gameDisplay, (147, 205, 218), False, path)
        else:
            for path in self.final_path:
                pygame.draw.lines(gameDisplay, (255,0,0), False, path)

    def backtrack(self, idx):
        node = self.unexplored_points[idx]
        parent = node[8]
        self.final_path.append(self.explored_paths[node[9]])
        while parent is not 0:
            node = self.unexplored_points[parent]
            parent = deepcopy(node[8])
            self.final_path.append(self.explored_paths[node[9]])
            
    def RRT_step(self, target, world):
        
        min_index = 0
        min_distance = np.inf
        for i, point in enumerate(self.unexplored_points):
            distance = np.sqrt((point[0] - target[0])**2 + (point[1] - target[1])**2)
            if distance<=min_distance:   #for ensuring that new nodes get more chance
                min_distance = distance
                min_index = i
            if np.sqrt((point[0] - self.goal[0])**2 + (point[1] - self.goal[1])**2) < self.reach_threshold:
                self.backtrack(i)
                return True
        

        # node = self.unexplored_points.pop(min_index)
        node = self.unexplored_points[min_index]

        point = [node[0], node[1]]
        heading = [node[2], node[3]]
        right_wheel = [node[4], node[5]]
        left_wheel = [node[6], node[7]]

        Vws = np.random.uniform(-40,40, (2,self.num_children))

        if np.random.uniform(0,1)<0.7:
            Vws[:, :self.num_children//2] = np.abs(Vws[:, :self.num_children//2])

        Ws = (Vws[0,:] - Vws[1,:])/(2*self.d)
        Vs = (Vws[0,:] + Vws[1,:])/2
        
        for i in range(self.num_children):
            heading_ = deepcopy(heading)
            point_ = deepcopy(point)
            right_wheel_ = deepcopy(right_wheel)
            left_wheel_ = deepcopy(left_wheel)
            
            samples = []
            samples_right = []
            samples_left = []

            num_samples = 50

            R = np.array([[np.cos(Ws[i]/num_samples), -np.sin(Ws[i]/num_samples)],
                          [np.sin(Ws[i]/num_samples), np.cos(Ws[i]/num_samples)]])
            
            prev_point = deepcopy(point)
            
            for j in range(num_samples):
                velocity = np.array(heading_)*Vs[i]/num_samples
                velocity_right = np.array(heading_)*Vws[0,i]/num_samples
                velocity_left = np.array(heading_)*Vws[1,i]/num_samples

                point_ = deepcopy([point_[0]+velocity[0], point_[1]+velocity[1]])
                right_wheel_ = deepcopy([right_wheel_[0]+velocity_right[0], right_wheel_[1]+velocity_right[1]])
                left_wheel_ = deepcopy([left_wheel_[0]+velocity_left[0], left_wheel_[1]+velocity_left[1]])
                # right_wheel_ = deepcopy((point_[0] + self.d*heading_[1], (point_[1] - heading_[0]*self.d)))
                # left_wheel_ = deepcopy((point_[0] - self.d*heading_[1], (point_[1] + heading_[0]*self.d)))

                samples.append(deepcopy(point_))
                samples_right.append(deepcopy(right_wheel_))
                samples_left.append(deepcopy(left_wheel_))

                heading_ = R@(np.array(heading_).T)
                heading_ = [heading_[0], heading_[1]]         
                if j%5 == 4:
                    if world.check_collision_holonomic(point_, prev_point):
                        break
                    elif j == num_samples-1:
                        self.explored_paths.append(samples)
                        self.unexplored_points.append([point_[0], point_[1], heading_[0], heading_[1]]+right_wheel_+left_wheel_+[min_index]+[len(self.explored_paths)-1])  
                        self.right_wheel_paths.append(samples_right)      
                        self.left_wheel_paths.append(samples_left)
        
        return False


class RobotNonHolonomic_tricycle_drive():
    def __init__(self, spawn, goal, heading = [1,0], num_children = 7, d = 4, B = 5):
        '''
        Non-holonomic tricycle drive robot

        spawn: initial position or World.spawn
        goal: target position
        num_children: number of children to protrude from each node of the RRT
        d: axle length/2
        B = robot length
        '''

        '''
        Using x_heading^2 + y_heading^2 = 1, 
        x_base^2 + y_base^2 = d^2, 
        y_base = (-x_heading)/(y_heading) x_base

        To solve for x_base and y_base
        '''

        # if not heading[1] == 0:
        self.base_l = [spawn[0] + d*heading[1], (spawn[1] - heading[0]*d)]
        self.base_r = [spawn[0] - d*heading[1], (spawn[1] + heading[0]*d)]
        self.front = [spawn[0] + B*heading[0], spawn[1] + B*heading[1]]

        self.pos = spawn
        self.goal = goal
        self.unexplored_points = [spawn+heading+self.base_r+self.base_l+self.front+[None]+[None]]
        self.num_children = num_children
        
        self.explored_paths = []
        self.right_wheel_paths = []
        self.left_wheel_paths = []
        self.front_wheel_paths = []

        self.reach_threshold = 5
        self.d = d
        self.B = B
        self.robot_size = max(2*d, B)
        self.final_path = []

    def print_paths(self, gameDisplay, type = 'center', done = False):
        if not done:
            if type == 'center':
                for path in self.explored_paths:
                    pygame.draw.lines(gameDisplay, (147, 205, 218), False, path)
            elif type == 'wheels':
                for path in self.right_wheel_paths:
                    pygame.draw.lines(gameDisplay, (255, 116, 0), False, path)
                for path in self.left_wheel_paths:
                    pygame.draw.lines(gameDisplay, (147, 205, 218), False, path)
        else:
            for path in self.final_path:
                pygame.draw.lines(gameDisplay, (255,0,0), False, path)

    def backtrack(self, idx):
        node = self.unexplored_points[idx]
        parent = node[10]
        self.final_path.append(self.explored_paths[node[11]])
        while parent is not 0:
            node = self.unexplored_points[parent]
            parent = deepcopy(node[10])
            self.final_path.append(self.explored_paths[node[11]])

    def RRT_step(self, target, world):
        
        min_index = 0
        min_distance = np.inf
        for i, point in enumerate(self.unexplored_points):
            distance = np.sqrt((point[0] - target[0])**2 + (point[1] - target[1])**2)
            if distance<min_distance:
                min_distance = distance
                min_index = i
            if np.sqrt((point[0] - self.goal[0])**2 + (point[1] - self.goal[1])**2) < self.reach_threshold:
                self.backtrack(i)
                return True
        
        # node = self.unexplored_points.pop(min_index)
        node = self.unexplored_points[min_index]

        point = [node[0], node[1]]
        heading = [node[2], node[3]]
        right_wheel = [node[4], node[5]]
        left_wheel = [node[6], node[7]]
        front = [node[8], node[9]]

        Vfs = np.random.uniform(-30,30, (1,self.num_children))
        alphas = np.random.uniform(-np.pi/6,np.pi/6, (1,self.num_children))
        
        # kinematics
        Ws = Vfs*np.sin(alphas)/self.B
        R = self.B/(np.tan(alphas)+0.00001)

        Vs = Ws*R
        Vls = Ws*(R+self.d)
        Vrs = Ws*(R-self.d)


        for i in range(self.num_children):
            heading_ = deepcopy(heading)
            point_ = deepcopy(point)
            right_wheel_ = deepcopy(right_wheel)
            left_wheel_ = deepcopy(left_wheel)
            front_ = deepcopy(front)
            
            samples = []
            samples_right = []
            samples_left = []
            samples_front = []

            num_samples = 20

            R = np.array([[np.cos(Ws[0,i]/num_samples), -np.sin(Ws[0,i]/num_samples)],
                          [np.sin(Ws[0,i]/num_samples), np.cos(Ws[0,i]/num_samples)]])
            R_alpha = np.array([[np.cos(alphas[0,i]), -np.sin(alphas[0,i])],
                          [np.sin(alphas[0,i]), np.cos(alphas[0,i])]])
            prev_point = deepcopy(point)
            
            for j in range(num_samples):
                velocity = np.array(heading_)*Vs[0,i]/num_samples
                velocity_right = np.array(heading_)*Vrs[0,i]/num_samples
                velocity_left = np.array(heading_)*Vls[0,i]/num_samples
                heading_front = R_alpha@(np.array(heading_).T)
                velocity_front = np.array(heading_front)*Vfs[0,i]/num_samples

                point_ = deepcopy([point_[0]+velocity[0], point_[1]+velocity[1]])
                right_wheel_ = deepcopy([right_wheel_[0]+velocity_right[0], right_wheel_[1]+velocity_right[1]])
                left_wheel_ = deepcopy([left_wheel_[0]+velocity_left[0], left_wheel_[1]+velocity_left[1]])
                front_ = deepcopy([front_[0]+velocity_front[0], front_[1]+velocity_front[1]])
                # right_wheel_ = deepcopy((point_[0] + self.d*heading_[1], (point_[1] - heading_[0]*self.d)))
                # left_wheel_ = deepcopy((point_[0] - self.d*heading_[1], (point_[1] + heading_[0]*self.d)))


                samples.append(deepcopy(point_))
                samples_right.append(deepcopy(right_wheel_))
                samples_left.append(deepcopy(left_wheel_))
                samples_front.append(deepcopy(front_))

                heading_ = R@(np.array(heading_).T)
                heading_ = [heading_[0], heading_[1]]         
                if j%5 == 4:
                    if world.check_collision_holonomic(point_, prev_point):
                        break
                    elif j == num_samples-1:
                        self.explored_paths.append(samples)
                        self.unexplored_points.append([point_[0], point_[1], heading_[0], heading_[1]]+right_wheel_+left_wheel_+front_+[min_index]+[len(self.explored_paths)-1])  
                        self.right_wheel_paths.append(samples_right)      
                        self.left_wheel_paths.append(samples_left)
                        self.front_wheel_paths.append(samples_front)

        # print(len(self.explored_paths))
        return False




class RobotHolonomic():
    def __init__(self, spawn, goal, heading = (1,0), num_children = 7, R = 5):
        '''
        Non-holonomic omnidirectional drive robot

        spawn: initial position or World.spawn
        goal: target position
        num_children: number of children to protrude from each node of the RRT
        R: Radius of the omnidirectional robot
        '''

        '''
        assuming 3 castor wheels and a 120 degree angle between adjacent pair of wheels
        '''
        
        '''
            HEADING IS NOT INCORPORATED INTO THE CODE  
        '''
        # if not heading[1] == 0:
        self.base_l = [spawn[0] - R*np.cos(np.pi/6), (spawn[1] - R*np.sin(np.pi/6))]
        self.base_r = [spawn[0] - R*np.cos(np.pi/6), (spawn[1] + R*np.sin(np.pi/6))]
        self.front = [spawn[0] + R, spawn[1]]

        self.pos = spawn
        self.goal = goal
        self.unexplored_points = [spawn+self.base_r+self.base_l+self.front+[None]+[None]]
        self.num_children = num_children
        
        self.explored_paths = []
        self.right_wheel_paths = []
        self.left_wheel_paths = []
        self.front_wheel_paths = []

        self.reach_threshold = 5
        self.R = R
        self.robot_size = 2*R
        self.final_path = []

    def print_paths(self, gameDisplay, type = 'center', done = False):
        if not done:
            if type == 'center':
                for path in self.explored_paths:
                    pygame.draw.lines(gameDisplay, (147, 205, 218), False, path)
            elif type == 'wheels':
                for path in self.right_wheel_paths:
                    pygame.draw.lines(gameDisplay, (255, 116, 0), False, path)
                for path in self.left_wheel_paths:
                    pygame.draw.lines(gameDisplay, (147, 205, 218), False, path)
        else:
            for path in self.final_path:
                pygame.draw.lines(gameDisplay, (255,0,0), False, path)

    def backtrack(self, idx):
        node = self.unexplored_points[idx]
        parent = node[8]
        self.final_path.append(self.explored_paths[node[9]])
        while parent is not 0:
            node = self.unexplored_points[parent]
            parent = deepcopy(node[8])
            self.final_path.append(self.explored_paths[node[9]])

    def RRT_step(self, target, world):
        
        min_index = 0
        min_distance = np.inf
        for i, point in enumerate(self.unexplored_points):
            distance = np.sqrt((point[0] - target[0])**2 + (point[1] - target[1])**2)
            if distance<min_distance:
                min_distance = distance
                min_index = i
            if np.sqrt((point[0] - self.goal[0])**2 + (point[1] - self.goal[1])**2) < self.reach_threshold:
                self.backtrack(i)
                return True
        
        # node = self.unexplored_points.pop(min_index)
        node = self.unexplored_points[min_index]
        point = (node[0], node[1])
        right_wheel = (node[2], node[3])
        left_wheel = (node[4], node[5])
        front = (node[6], node[7])

        Vs = np.random.uniform(-30,30, (2,self.num_children))
        # Vs = np.array([[10],[0]])

        for i in range(self.num_children):
            
            point_new = [point[0]+Vs[0,i], point[1]+Vs[1,i]]
            right_wheel_new = [right_wheel[0]+Vs[0,i], right_wheel[1]+Vs[1,i]]
            left_wheel_new = [left_wheel[0]+Vs[0,i], left_wheel[1]+Vs[1,i]]
            front_new = [front[0]+Vs[0,i], front[1]+Vs[1,i]]

            if world.check_collision_holonomic(point, point_new):
                break
            else:
                self.explored_paths.append((point, point_new))
                self.unexplored_points.append(point_new+right_wheel_new+left_wheel_new+front_new+[min_index]+[len(self.explored_paths)-1])  
                self.right_wheel_paths.append((right_wheel, right_wheel_new))      
                self.left_wheel_paths.append((left_wheel, left_wheel_new))
                self.front_wheel_paths.append((front, front_new))

        # print(len(self.explored_paths))
        return False