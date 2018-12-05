from __future__ import division
import numpy as np
from pprint import pprint
import random
import time
import csv
##1)Main algorithm, 2), 3), 4) benchmarks, 5
##1) python tester.py test_maze_01.py a_star_dinamic_optimizer
##2) python tester.py test_maze_01.py random
##3) python tester.py test_maze_01.py random_detector
##4) python tester.py test_maze_01.py random_detector_visited
##5) python tester.py test_maze_01.py a_star_dinamic_a_star
## after model name you can pass exploration limit


# Dir of the heading
dir_sensor = {'u':['l','u','r'],
              'r':['u','r','d'],
              'd':['r','d','l'],
              'l':['d','l','u'],
              'up':   ['l','u','r'],
              'right':['u', 'r','d'],
              'down': ['r', 'd', 'l'],
              'left': ['d','l','u']}     
              
dir_move = {'up':[-1, 0], 'right':[0,1], 'down': [1, 0], 'left':[0, -1],
            'u':[-1, 0], 'r':[0, 1], 'd':[1, 0],'l':[0, -1]}  
            
allowed_up    = [1, 3, 5, 7, 9, 11, 13, 15]
allowed_right = [2, 3, 6, 7, 10, 11, 14, 15]
allowed_down  = [4, 5, 6, 7, 12, 13, 14, 15]
allowed_left  = [8, 9, 10, 11, 12, 13, 14, 15]

dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}
                   
rotate_action = ['left', 'front', 'right']

delta = [[-1, 0],  ## Up          
         [ 0, 1],  ## Right      
         [ 1, 0],  ## Down
         [ 0,-1]]  ## Left
heading =['up', 'right', 'down', 'left']
##up right down left
path_reference = {'^': 0, '>':1, 'v':2, '<':3}

policy_dict = {'^':'u', '>':'r', 'v':'d', '<':'l'}
policy_simbol = ['^', '>', 'v', '<']


g_cost = {0:1, 1:0.99, 2:1}

class Robot(object):
    def __init__(self, maze_dim, model, verbo=False, exploration_limit=70):
        ## random.seed(136)
        
        # Type of model
        self.model = model
        self.maze_dim = maze_dim
        self.verbose = verbo
        
        ## Initial Position
        self.location = [self.maze_dim-1, 0]
        self.heading = 'up'
        
        ## Time
        self.step_count = 0
        self.run = 0
        
        ## Exploration measurement
        self.exploration_count = 0 
        self.maze_area = self.maze_dim * self.maze_dim
        self.exploration_ratio = 0
        self.exploration_limit = int(exploration_limit)
        
        ## Testter comunication
        self.max_time = 0
        self.tester_loc = 0
        self.tester_step = 0
        self.tester_bin_wall = 0
        
        ## Evaluation: time, goalfound(true/false), score
        self.score = 0
        
        ## Goal definition
        ## this goal location is the objective of the first run, our robot must find out the location and save it in the goal_location var
        self.goal_location_explore = 0
        self.goal_location_exploit = 0
        self.found_goal_step_explore = 0
        self.found_goal_step_exploit = 0
        self.found_goal = False
        self.found_goal_exploit = False
        self.goal_bounds = [self.maze_dim/2-1, self.maze_dim/2]
        
        ## back of the mouse if zero mean 'wall', if one free of 'wall', initiate with wall behind
        self.back_sense = 0 
        self.reverse_value = 0
        
        ## Heuristics
        self.g = 0
        self.first_heuristic = [[int(self.calculate_heuristic(col, row)) for col in range(self.maze_dim)] for row in range(self.maze_dim)]
        
        ##Exploring
        self.binary_tile = [[0 for col in range(self.maze_dim)]for row in range(self.maze_dim)]
        self.path = [[' ' for col in range(self.maze_dim)] for row in range(self.maze_dim)]
        self.path2 = [[' ' for col in range(self.maze_dim)] for row in range(self.maze_dim)]
        self.visited = [[ 0 for col in range(self.maze_dim)] for row in range(self.maze_dim)]
        self.visited[self.location[0]][self.location[1]] += 1
        self.g_cost = [[ 0 for col in range(self.maze_dim)] for row in range(self.maze_dim)]
        ##Dinamic programming
        self.q_value = 0
        self.policy = [[' ' for col in range(self.maze_dim)] for row in range(self.maze_dim)]
        self.dead_ends = set()
        self.dead_ends_second_list = set()

    def report(self):
        ## explore=True
        ## if explore == True:
        ## 'Steps: ', self.step_count, '/ ',self.max_time,']'
        print'##### Explore Report ####'
        if self.found_goal:
            print'Explore Found Goal in location', self.goal_location_explore, 'in step:', self.found_goal_step_explore
        print 'Exploration proportion: %s of %s tiles Percentage explored:%s ' % (self.exploration_count, self.maze_area, self.exploration_ratio)
        print 'Current Location: ', self.location, 'Heading: ', self.heading 
        print'----First Heuristic used for exploration----'
        print np.array(self.first_heuristic)
        print'Path made in exploration'
        print np.array(self.path)
        print'Binary Tile Grid dicover'
        print np.array(self.binary_tile)
        print 'Visited grid made' 
        print np.array(self.visited)
    ## else:
        print '####### Exploit Report ######'
        if self.found_goal_exploit:
            print 'Exploit Found Goal in location', self.goal_location_exploit, 'in step:', self.found_goal_step_exploit
        print('--- Q_value----')
        print np.array(self.q_value)
        print '---Policy---'
        print np.array(self.policy)
        print '----Path made---'
        print np.array(self.path2)

    def next_move(self, sensors):
    ## This function comunicate with tester, tester comunicate with maze
        if self.run == 0 :
            rotation, movement = self.explore(sensors, self.model)   
    
        elif self.run == 1:
            rotation, movement = self.exploit(sensors)
      
        return rotation, movement   
          
    def rotate(self, direction):   
        ''' direction is an int 0, 1, 2 that correspont to left front right of the current heading'''
        rotation_index = [-90, 0, 90]
        direction_index = {'left': 0, 'front':1, 'right':2}
        ## orientation index 0 left 1 front 2 right
        orientation = direction_index[direction]
        ## new_direction is on of u, r, d, l key of dir_move
        new_heading = dir_sensor[self.heading][orientation] 
        # change heading and position in our robot class
        self.heading = new_heading
        # call the value in degrees to pass to the tester class
        next_rotation = rotation_index[orientation]
        
        return next_rotation
        
    def move(self, movement=1, exploration=False):
        ''' movement is  a quantiy, modify our grid in a scale of 2 
        and pass the regular movement to the tester'''  
        ## Get current coordenates
        x = self.location[0]
        y = self.location[1]  
        ## get the direction to the actual heading 
        delta_x = dir_move[self.heading][0]
        delta_y = dir_move[self.heading][1]
        # Sum the movement to the current cordenates
        x2 = x + delta_x*movement
        y2 = y + delta_y*movement
        ## if the grid is free of wall in the next position
        if self.check_wall(self.heading) == False:
            ## check if is it inside the grid
            if self.check_grid_boundry(x2,y2):
                ## assign new location            
                self.location[0] = x2 
                self.location[1] = y2
                ## if flag exploration is true(exploration phase) save the moves in path
                if exploration == True:
                    ## path grid is for marking the path in the exploration phase
                    self.path[x][y] = self.graph_move(self.heading)
                    self.path[x2][y2] = 'M'
                elif exploration == False:
                    ## path 2 is fo_r marking the path in the exploration phase
                    self.path2[x][y] = self.step_count
                    #self.graph_move(self.heading)
                    self.path2[x2][y2] = 'M'
                ## remember how many times you been in the tile
                self.visited[x2][y2] +=1
                ## we have not wall behind
                self.back_sense = 1
            else:
                ## we are out of the limits of the grid
                ## print('Out of the limits of the grid')
                movement = 0
        else:
            pass
            ## print('There is a wall in front direction of the heading: {}, requiered movement = {} at step_count = {}, in current location = {}  on binary tile = {}'.format(
            ## self.heading, movement, self.step_count, self.location, self.binary_tile[self.location[0]][self.location[1]]))
            
        return movement                            
            
    def construc_binary_tile(self, sensors):
        sensor_active = self.check_sensor(sensors)
       
        if self.heading == 'up' or self.heading == 'u':
            ## left, front, right, back 
            bin_tile = sensor_active[0]*8 + sensor_active[1]*1 + sensor_active[2]*2 + self.back_sense*4
        if self.heading == 'right' or self.heading == 'r':
            ## left, front, right, back
            bin_tile = sensor_active[0]*1 + sensor_active[1]*2 + sensor_active[2]*4 + self.back_sense*8
        if self.heading == 'down' or self.heading =='d':
            ## left, front, right, back
            bin_tile = sensor_active[0]*2 + sensor_active[1]*4 + sensor_active[2]*8 + self.back_sense*1
        if self.heading == 'left' or self.heading == 'l':
            ## left, front, right, back
            bin_tile = sensor_active[0]*4 + sensor_active[1]*8 + sensor_active[2]*1 + self.back_sense*2
        
        self.binary_tile[self.location[0]][self.location[1]] = bin_tile
        return None
        
    def check_sensor(self, sensors):
        sensor_active = [0,0,0]
        for i in range(len(sensors)):
            if sensors[i] > 0:
                sensor_active[i] = 1
        return sensor_active
        
    def check_exploration_ratio(self, limit= None):
        if limit is None:
            limit = 70.
        if self.exploration_count/self.maze_area*100 >= limit:
            return True
        else:
            False
            
    def check_wall(self, heading, x=None, y=None):
        if x is None and y is None:
            x = self.location[0]
            y = self.location[1]
        ## return false if there is not wall in the direction of the heading
        if   heading == 'u' or heading == 'up':
           if self.binary_tile[x][y] in allowed_up:
               return False
           else:
               return True    
        elif heading == 'r' or heading == 'right':
           if self.binary_tile[x][y] in allowed_right:
               return False
           else:
               return True 
        elif heading == 'd' or heading == 'down':
           if self.binary_tile[x][y] in allowed_down:
               return False
           else:
               return True    
        elif heading == 'l' or heading == 'left':
           if self.binary_tile[x][y] in allowed_left:
               return False
           else:
                return True
                
    def check_grid_boundry(self, x2, y2):
        if (x2 >= 0  and x2 <= (self.maze_dim-1) and y2 >= 0 and y2 <= (self.maze_dim-1)):
           return True
        else:
            return False
    ## this is for using in the exploiting phase        
    def check_possibles_movement(self, sensors, rotate_dir):
        ## print(sensors) 
        movement = 1
        while movement < 3:
            ## while
            if movement == sensors[rotate_dir]:
                break
            else:
                movement += 1
            
        ## print(movement, 'movement')
        return movement
     
            
      
    def graph_move(self,heading):
        if heading == 'u' or heading == 'up':
            return '^'
        if heading == 'r' or heading =='up':
            return '>'
        if heading == 'd' or heading == 'down':
            return 'v'
        if heading == 'l' or heading == 'left':
            return '<' 
            
    def check_goal(self, x=None, y=None, check_visited=False):
        ## if nothig is passed as x and y use the current location for the check
        if x is None and y is None:
            x = self.location[0]
            y = self.location[1]
        
        if x in self.goal_bounds and y in self.goal_bounds:
            ## we check first visit on explration method
            if check_visited:
            ## visited must be one and not zero becouse we check the goal after the move method that chage the visited grid 
                if self.visited[x][y] == 1:
                    return True
                else:
                    return False 
             ## when visited is not required this return true       
            else:
                return True
        ## when x and y are not in goal bounds=[maze_dim-1, maze_dim]        
        else:
            return False
            
                
    def save_goal(self, x=None, y=None, explore=False):
      ## Save the location of the goal for later uses
        if x is None and y is None:
            x = self.location[0]
            y = self.location[1]  
        if explore:
            
            self.goal_location_explore = [x, y]
            ## print('saving', self.location, 'in', self.goal_location_explore)
            self.found_goal_step_explore = self.step_count
            self.found_goal = True
        else:
            self.goal_location_exploit = [x, y]
            self.found_goal_step_exploit = self.step_count    
            self.found_goal_exploit = True
            return None
    
    def save_to_file(self, file_name, runtime_run1, score):
        
        fields = [self.found_goal, self.found_goal_step_explore, runtime_run1, self.exploration_ratio, \
                  self.found_goal_exploit, self.found_goal_step_exploit, score]
        
        with open(file_name, 'a') as file_:
            writer = csv.writer(file_)
            writer.writerow(fields)        
            
    ## def random_model(self):
        ## random_action = rotate_action[random.randrange(0,3)]
        ## rotation = self.rotate(random_action)
        ## movement = self.move() 
        
        ## return  rotation, movement
        
    def a_star(self, sensors, cost=1, heuristic_grid=None, exploration=False):
        
        if heuristic_grid == None:
            heuristic_grid = self.first_heuristic
        
        x = self.location[0]
        y = self.location[1]
        ## unpack the measurement of the detector, true or false, and one or two the number of movement for reverse
        ## self.verbo(self.detect_dead_end())
        if self.detect_dead_end():
            ## self.verbo('Dead_end Detected')   
            ## self.verbo([self.dead_ends, ' Dead end list'])
            ## reverse go back 1 or 2 tile depending of the deepness of the dead end
            movement = self.reverse(self.reverse_value)
            ## self.verbo(['reverse', self.reverse_value]) 
            ## rotate nothing, return 0
            rotation = 0
            ## print( 'Rotation: ',rotation,'Movement: ' ,movement)
            return rotation, movement
        else:  
            sensor_active = self.check_sensor(sensors)
            descarted_for_dead_end = False
            self.verbo(sensor_active)
            ## This list will be generated in every movement call from the tester, this means that we choose
            ## possibles action in the current location, not from all location as in a pure algorith of search
            open_list = []
            self.verbo([self.location, self.heading, sensors])
            ## This for loop generates the possibles next tiles and append it to the list
            for i in range(len(sensor_active)):
                self.verbo([len(sensor_active), 'Len  of the sensor_active'])
                self.verbo(['state_sensor', sensor_active[i],'index', i,'position', self.location, 'heading', self.heading])
                ## filter all zero sensor measurement
                if sensor_active[i] == 1:
                    ## current coordenate plus the direction of move of each sensor for our current heading
                    x2 = x + dir_move[dir_sensor[self.heading][i]][0]
                    y2 = y + dir_move[dir_sensor[self.heading][i]][1]
                    self.verbo(['candidates',x2,y2])
                    ## Filter possibles moves   
                    if self.check_grid_boundry(x2, y2): 
                        
                        self.verbo(['Candidates for neighbour:', [x2, y2], 'step_count', self.step_count])
                        ## self.g_cost[x2][y2] = g2
                        ## catch the number of visited that we have made in the next tile
                        v = self.visited[x2][y2] 
                        ## calculate distance to the goal point
                        h = heuristic_grid[x2][y2]
                        ## assign a bigger cost to index that represent turn lef or right, index 1 (front) have lower cost
                        g2 = g_cost[i]
                        ## g2 = 1
                        f = h + g2
                        if exploration == True:
                        ## this method check, if we are in 5 tile up a 4 and down a one, if we are in 10 tile right an 8 and left an 2
                            self.check_memory_for_dead_end()
                            if tuple([x2, y2]) not in self.dead_ends or tuple([x2, y2]) not in self.dead_ends_second_list:
                                open_list.append([v,f, x2, y2, i])    
                            else:
                                self.verbo([[x2, y2], 'Descarted by the dead end list'])
                                descarted_for_dead_end = True
                        ## exploration false mean we are in exploit phase
                        elif exploration == False:
                            ## if exploration is false check only one list of the dead ends
                            if tuple([x2, y2]) not in self.dead_ends:
                                open_list.append([h, v, x2, y2, i])       
                            else:
                               self.verbo([[x2, y2], 'Descarted by the dead end list'])
                               descarted_for_dead_end = True    
                    else:
                        self.verbo([[x2,y2],' Descarted by being  outside the grid'])       
            ## When we descart a posible neighbour from the list  becouse it is in the the dead_end list is becouse we already been here.
            ## And for that we know that that step guide us to an dead end and for that we descard it. But this could be leave our list of neighbour empty.
            ## and if that happen we reerse one.    
            if descarted_for_dead_end:
                 if len(open_list) == 0:
                     if self.check_wall_behind():
                         rotation =  self.rotate('front')
                         movement =  self.reverse(1)
                         return rotation, movement
            open_list.sort()
            open_list.reverse() 
            self.verbo([[open_list[i][2:4] for i in range(len(open_list))], 'Open List'])
            next_move = open_list.pop()
            
            ## unpack index of rotation
            if exploration == True:
                rotate_dir = next_move[4]
            
            elif exploration == False:
                ## update g cost
                self.g = next_move[2]
            ## when we are exploiting we change the order of the priorities
                rotate_dir = next_move[4]
            ## update the rotation and return 90, 0 or -90 for passing to the tester
            self.verbo(['current heading', self.heading, 'location', self.location,'tile', self.binary_tile[self.location[0]][self.location[1]], 'rotate dir', rotate_dir])
            rotation = self.rotate(rotate_action[rotate_dir]) 
            ## execute and move in tha new direction
            if exploration == True:
               movement = self.move(movement=1, exploration=True)    
            elif exploration == False:
                ## movement_ = self.check_possibles_movement(sensors, rotate_dir)
                movement_ = 1
                movement = self.move(movement=movement_)
            self.verbo(['next heading', self.heading, 'next_location', self.location, 'next_tile', self.binary_tile[self.location[0]][self.location[1]]])    
            return rotation, movement
        
    def random_search(self, sensors, cost=1, d_end_detector=False, visited_count=False, exploration=False):
        x = self.location[0]
        y = self.location[1]
        ## unpack the measurement of the detector, true or false, and one or two the number of movement for reverse
        ## self.verbo(self.detect_dead_end())
        ## if dead end detector is true
        # Optional
        if d_end_detector and self.detect_dead_end():
                self.verbo(['location', self.location,'heading', self.heading, 'step', self.step_count])
                self.verbo('Dead_end Detected')   
                self.verbo([self.dead_ends, ' Dead end list']) 
                ## reverse go back 1 or 2 tile depending of the deepness of the dead end
                movement = self.reverse(self.reverse_value)
                self.verbo(['reverse', self.reverse_value]) 
                ## rotate nothing, return 0
                rotation = 0
                self.verbo([ 'Rotation: ',rotation,'reverse: ' ,movement,'bin', self.binary_tile[self.location[0]][self.location[1]]])
                
                return rotation, movement
        
        else:  
            sensor_active = self.check_sensor(sensors)
            ## flag for when we descard some neighbour from list for being a dead end
            descarted_for_dead_end = False
            ## self.verbo(sensor_active)
            ## This list will be generated in every movement call from the tester, this means that we choose
            ## possibles action in the current location, not from all location as in a pure algorith of search
            open_list = []
            ## This for loop generates the possibles next tiles and append it to the list
            self.verbo(['sensor', sensor_active,'loc' ,self.location, 'head',self.heading])
            for i in range(len(sensor_active)):
                ## self.verbo([len(sensor_active), 'Len  of the sensor_active'])
                ## this filter implies that we have not wall in front
                if sensor_active[i] == 1:
                    ## current coordenate plus the direction of move of each sensor for our current heading
                    x2 = x + dir_move[dir_sensor[self.heading][i]][0]
                    y2 = y + dir_move[dir_sensor[self.heading][i]][1]
                    ## Filter possibles moves   
                    if self.check_grid_boundry(x2, y2): 
                        self.verbo(['Candidates for neighbour:', [x2, y2]])
                        ## catch the number of visited that we have made in the next tile
                        v = self.visited[x2][y2] 
                        ## Optional: model with dead end detection
                        if d_end_detector:
                            if exploration == True:
                                ## create a second list with more dead end, if the tile is 5 check in up direction a 4 and down a 1, same for 10, rigth 8  and left 2.
                                self.check_memory_for_dead_end()
                                ## check if the tuple is not in the dead end list
                                if tuple([x2, y2]) not in self.dead_ends or tuple([x,y2]) not in self.dead_ends_second_list:
                                    open_list.append([x2, y2, i])
                                else:
                                    self.verbo([[x2, y2], 'Descarted by the dead end list'])
                                    descarted_for_dead_end = True             
                            elif exploration == False:
                                if tuple([x2, y2]) not in self.dead_ends:
                                    open_list.append([x2, y2, i])
                                else:
                                    self.verbo([[x2, y2], 'Descarted by the dead end list'])
                                    descarted_for_dead_end = True            
                       ## Optional: model with visited_count and dead end detection
                        elif d_end_detector and visited_count:
                            if exploration == True:
                                self.check_memory_for_dead_end()
                                if tuple([x2, y2]) not in self.dead_ends or tuple([x,y2]) not in self.dead_ends_second_list:
                                    open_list.append([v, x2, y2, i])
                                else:
                                    self.verbo([[x2, y2], 'Descarted by the dead end list'])
                                    descarted_for_dead_end = True    
                            elif exploration == False:
                                if tuple([x2, y2]) not in self.dead_ends:
                                    open_list.append([v, x2, y2, i])
                                else:                 
                                    self.verbo([[x2, y2], 'Descarted by the dead end list'])
                                    descarted_for_dead_end = True    
                        ## simple random model without dead_end check and without visited, only check boundry grid
                        else:
                           open_list.append([x2, y2, i]) 
                    else:
                        self.verbo([[x2,y2],' Descarted by being  outside the grid'])       
            
           
            ## if is the base random model and the list is empty choose a random action
            ## if len(open_list) == 0 and d_end_detector == False and visited_count == False:
                ## random_action = rotate_action[random.randrange(0,3)]
                ## rotation = self.rotate(random_action)
                ## movement = self.move() 
            
                ## return  rotation, movement
            ## if the list is empty becouse we descard one dead end do automatic reverse one movement included in the dead_end detector model
            if descarted_for_dead_end:
                 if len(open_list) == 0:
                     if self.check_wall_behind():
                        rotation =  self.rotate('front')
                        movement =  self.reverse(1)
                    
                        return rotation, movement
            ## we add one element to the tuple for random assigment of priority,roandom model:4 elements, d_end_detector:4 element, visites and d_end detector:5 element
            if visited_count == True:    
                for i in open_list:
                    i.insert(0, random.randint(0, len(open_list)))
            
            else:
                for i in open_list:
                    i.insert(0, random.randint(0, len(open_list)))
                
            open_list.sort()
            open_list.reverse()
            self.verbo([[open_list[i][2:4] for i in range(len(open_list))], 'Open List'])
            
            try: 
                next_move = open_list.pop()
            except IndexError:
                ## emergecy decision :)
                random_action = rotate_action[random.randrange(0,3)]
                rotation = self.rotate(random_action)
                movement = self.move() 
                return rotation, movement 
            
            ## unpack index of rotation
            ## dead end detector model
            if d_end_detector:
                ##
                rotate_dir = next_move[3]
            ## dead end detector plus visited count                
            elif d_end_detector and visited_count:
                ## 4
                rotate_dir = next_move[4]
            else:
            ## normal model    
                rotate_dir = next_move[3]
        
            ## update the rotation and return 90, 0 or -90 for passing to the tester
            rotation = self.rotate(rotate_action[rotate_dir]) 
            ## execute and move in tha new direction
            if exploration == True:
                movement = self.move(movement=1, exploration=True)    
            elif exploration == False:
                ## check quantity of possibles moves without walls
                movement_ = self.check_possibles_movement(sensors, rotate_dir)
                ## randomize a quantity between the maximun and zero
                movement_2 = random.randint(0,movement_)
                ## execute the movement
                movement = self.move(movement=movement_2)
        
            return rotation, movement            
                           
        
    def construct_optimal_path(self, sensors):
        ## this method construct the action needed to follow the policy generated by the method dinamic value generator
        ## we recive directions up right down and left and we have to traslate to rotation
        ## and them move
        x = self.location[0]
        y = self.location[1]
        ## initialize movement in one
        movement_ = 1
        
        if self.policy[x][y] != '*' and self.policy[x][y] != ' ':
            ## unpack heading its an  ^, >, < or v symbol, policy dict convert to u, r, d, l key
            direction_policy = policy_dict[self.policy[x][y]]
            ## given the self.heading and the direction of the policy return me the index of the correspondent rotation
            index_rotate = dir_sensor[self.heading].index(direction_policy)
            ## execute rotation change the self.heading
            ## get the rotation for passing to the tester
            rotation = self.rotate(rotate_action[index_rotate])
        
            new_heading = self.heading
            ## debugging
            ## self.verbo(['tester', self.tester_loc, 'tester_bin_wall:', self.tester_bin_wall[self.tester_loc[0]][self.tester_loc[1]], 'tester_step:', self.tester_step]) 
            ## self.verbo(['sensors:', sensors])
            ## self.verbo(['heading:', self.heading, 'step:', self.step_count, 'location:', self.location, 'binary:', self.binary_tile[self.location[0]][self.location[1]]])
            ## self.verbo(['location:', self.location, 'direction policy:', direction_policy, 'index-rotate:', index_rotate, 'rotation:', rotation, 'new_heading:', new_heading,'step:', self.step_count])
            ## move a maximun of 3 steps in the direction
            while movement_ < 3:       
                ##unpack the symbol of the current position indicated by the policy :^ ,<, > or v
                policy_symbol_ = self.policy[x][y]
                self.verbo(['current policy symbol:', policy_symbol_])
                ## move one in the recently roated heading
                x += dir_move[self.heading][0]
                y += dir_move[self.heading][1]
                ## if the simbol in the new location is the same, move one more
                if self.policy[x][y] == policy_symbol_:
                    self.verbo(['next policy symbol:', self.policy[x][y]])
                    movement_ +=1
                else:
                    self.verbo(['next policy symbol:', self.policy[x][y]])
                    break
     
            self.move(movement=movement_)
            self.verbo(['movement done:', movement_])
            return rotation, movement_
        else:
            return 0, 0
    
        
    def dinamic_value_generator(self, cost_step=1):
        ## Part of this code is provided in the curse of robotics by Udacity.
        value_to_goal = [[ 99 for col in range(self.maze_dim)]for row in range(self.maze_dim)]
        change = True
        while change:
            change =False
            ##loop in all the coordenates
            for x in range(self.maze_dim):
                for y in range(self.maze_dim):
                    ## set the value of the goal to zero
                    if  self.check_goal(x,y):
                        if value_to_goal[x][y] > 0:
                            value_to_goal[x][y] = 0
                            self.policy[x][y] = '*'
                            change = True
                    ## for all other x and y calculate the values
                    else:
                        for i, direction in enumerate(heading):
                            ## check for walls before continue, if there is not wall in that coordenate and direction
                            ## checkgoal call self.binary_tile
                            #time.sleep(1);print('checking for walls')
                            if self.check_wall(direction, x, y) == False:
                                x2 = x + dir_move[direction][0]
                                y2 = y + dir_move[direction][1]
                                #time.sleep(1);print('checking for boundrys')
                                if self.check_grid_boundry(x2, y2):
                                    value2 = value_to_goal[x2][y2] + cost_step
                                    ##  
                                   
                                    if value2 < value_to_goal[x][y]:
                                        change =True
                                        value_to_goal[x][y] = value2
                                        ## if self.policy[x][y] == ' ':
                                        self.policy[x][y] = policy_simbol[i]
                                        ## else:
                                            
                                            ## pass
                                        #time.sleep(1);print('value is lower that other')    
        return value_to_goal
                                   
    def calculate_heuristic(self, x, y):  
        goal_x = self.maze_dim/2
        goal_y = self.maze_dim/2
        ##min(x-6), (x-7) + min(y-6),y-7, real index are 5 and 6
        distance =  min(abs(x-goal_x),abs(x-goal_x+1)) + min(abs(y-goal_y), abs(y-goal_y+1))
        ## (abs(x-goal_x)+ abs(x-goal_x+1) + abs(y-goal_y) + abs(y-goal_y+1))/2
         ## 
         ##
        return distance
    
    ## def calculate_manhattan_distance(self, x, y):
        ## goal_x = self.maze_dim/2
        ## goal_y = self.maze_dim/2
        ## distance = abs(goal_x -x) + abs(goal_y-y)
        ## return distance
        
        
    
    
    def check_wall_behind(self):
         reverse_direction = dir_reverse[self.heading]
         x = self.location[0]
         y = self.location[1]
         
         if reverse_direction == 'up'or reverse_direction == 'u':
            if self.binary_tile[x][y] in allowed_up:
                return True
            else:
                False
         if reverse_direction == 'right'or reverse_direction == 'r':
            if self.binary_tile[x][y] in allowed_right:
                 return True
            else:
                 return False   
         if reverse_direction == 'down' or reverse_direction == 'd':
            if self.binary_tile[x][y] in allowed_down:
                return True
            else:
                return False
         if reverse_direction == 'left' or reverse_direction == 'l':
            if self.binary_tile[x][y] in allowed_left:
                return True
            else:
                return False
    
    
    
    def reverse(self, movement):
        x = self.location[0]
        y = self.location[1]
    
        for i in range(movement):
            x2 = x + dir_move[dir_reverse[self.heading]][0]
            y2 = y + dir_move[dir_reverse[self.heading]][1]
            self.location[0] = x2
            self.location[1] = y2
            self.path[x][y] = 'R'
            self.path[x2][y2] = 'M'
            self.visited[x2][y2] += 1
            x = x2
            y = y2
        return -movement
        
        
            
    def reset_robot(self):
        self.location = [self.maze_dim-1,0]
        self.heading = 'up'
        self.step_count = 0
        ## self.found_goal = False
        self.found_goal_step = 0
        return None
        
        
    def detect_dead_end(self):
        ## For the current heading
        if self.heading == 'up' or self.heading ==  'u':
            ## if the binary tile is dead end
            if self.binary_tile[self.location[0]][self.location[1]] == 4:
                ## if the tile behind the current tile is a paralel vertical tile
                if self.binary_tile[self.location[0]+1][self.location[1]] == 5:
                    ## append the two tile to the the dead end list
                    self.dead_ends.add(tuple(self.location))
                    self.dead_ends.add(tuple([self.location[0]+1, self.location[1]]))
                    
                    self.reverse_value =2
                    return True
                else:
                    ## append only the current tile to the dead end list and return 1
                     self.verbo('apending to dead end list')
                     self.dead_ends.add(tuple(self.location))
                     self.reverse_value =1
                     return True
            ## else the tile is other value than 4 return False         
            else:
                return False
        elif self.heading == 'down' or self.heading == 'd':
            if self.binary_tile[self.location[0]][self.location[1]] == 1:
                if self.binary_tile[self.location[0]-1][self.location[1]] == 5:
                    ## if self.binary_tile[self.location[0]-2][self.location[1]] == 5:
                        
                    self.dead_ends.add(tuple(self.location))
                    self.dead_ends.add(tuple([self.location[0]-1, self.location[1]]))
                    self.reverse_value =2
                    return True
                else:
                    self.dead_ends.add(tuple(self.location))
                    self.reverse_value =1
                    return True
            else:
                return False
        elif self.heading =='right' or self.heading == 'r':
            if self.binary_tile[self.location[0]][self.location[1]] == 8:
                if self.binary_tile[self.location[0]][self.location[1]-1] == 10:
                    self.dead_ends.add(tuple(self.location))
                    self.dead_ends.add(tuple([self.location[0], self.location[1]-1]))
                    self.reverse_value =2
                    return True
                else:
                    self.dead_ends.add(tuple(self.location))
                    self.reverse_value =1
                    return True
            else:
                return False
        elif self.heading == 'left' or self.heading == 'l':
            if self.binary_tile[self.location[0]][self.location[1]] == 2:
                if self.binary_tile[self.location[0]][self.location[1]+1] == 10:
                    self.dead_ends.add(tuple(self.location))
                    self.dead_ends.add(tuple([self.location[0], self.location[1]+1]))
                    self.reverse_value = 2
                    return True
                else:
                    self.dead_ends.add(tuple(self.location))
                    self.reverse_value = 1
                    return True
            else:
                return False
         
    def check_memory_for_dead_end(self):
        ## this funtion add more tiles to the dead end list
        x = self.location[0]
        y = self.location[1]
        ## if we are in 5 or 10 tile we must be alert becouse a dead end could be near, and for prevention we try to remember
        if self.binary_tile[x][y] == 5:
            try:
                 ## Look in the upper direction
                x2 = x + delta[0][0]
                y2 = y + delta[0][1]
                ## Look in the lower direction
                x3 = x + delta[2][0]
                y3 = y + delta[2][1]
                ## this return and IndexError if we dont visited yet the tile
                ## we look if the upper tile is a four or if the lower tile is a one, if that true put the current tile in the dead_end list too
                if self.binary_tile[x2][y2] == 4:
                    self.dead_ends_second_list.add(tuple(self.location))
                elif self.binary_tile[x3][y3] == 1:
                    self.dead_ends_second_list.add(tuple(self.location))
                else:
                    pass 
            except IndexError:
                print ' Tile dont visited yet we dont have it in memory'
                
        elif self.binary_tile[x][y] == 10:
            try:
                ## Look in the right direction
                x2 = x + delta[1][0]
                y2 = y + delta[1][1]
                ## Look in the left direction
                x3 = x + delta[3][0]
                y3 = y + delta[3][1]
                
                if self.binary_tile[x2][y2] == 8:
                   self.dead_ends_second_list.add(tuple(self.location))
                elif self.binary_tile[x3][x3] == 2:
                   self.dead_ends_second_list.add(tuple(self.location))
                else:
                    pass
            except IndexError:
                print ' Tile dont visited yet we dont have it in memory'
        
    def explore(self,sensors, model):
        ## if the binary tile is not set up yet, set it up
        if self.binary_tile[self.location[0]][self.location[1]] > 0:
            pass 
        else:
            self.construc_binary_tile(sensors)
            self.exploration_count +=1  
            ##update exploration ratio
            self.exploration_ratio = self.exploration_count/self.maze_area*100.
        ## If we are in the goal location, save it in the robot memory
        if self.check_goal(check_visited=True):
            self.step_count +=1
            ## save location and set found goal flag to True
            self.save_goal(explore=True)
            ## print('Found Goal in location {} in the step {}, exploration percentage {}\n'.format(self.goal_location_explore, self.found_goal_step_explore, self.exploration_ratio))
        if self.found_goal:    
            ## if found the goal and exploration is above 60% reset the robot and pass the run
            if self.exploration_ratio >= self.exploration_limit:
            ## if  self.check_exploration_ratio(limit=90.): 
                print 'Reseting robot beggining next phase\n'
                ## self.report()
                rotation, movement = 'Reset', 'Reset'
                ## robot return to initial position and heading
                self.reset_robot()
                self.run = 1
                
                
                return rotation, movement
        ## use the model selected 
        if self.model[0:6] == 'a_star':
            rotation, movement = self.a_star(sensors, exploration=True)
        
        elif self.model == 'random':
            rotation, movement = self.random_search(sensors, exploration=True)            
        
        elif self.model == 'random_detector':
            rotation, movement = self.random_search(sensors, d_end_detector=True, exploration=True)            
        
        elif self.model == 'random_detector_visited':
            rotation, movement = self.random_search(sensors, d_end_detector=True, visited_count=True, exploration=True)           
        
        else:
            print 'Do not match any model name'
            return 0, 0 
        
        self.step_count +=1
        
        return rotation, movement   
    
    def exploit(self, sensors):
        ## print(self.model, type(self.model), 'exploit')
        if self.binary_tile[self.location[0]][self.location[1]] > 0:
            pass 
        else:
            self.construc_binary_tile(sensors)


        if self.model == 'a_star_dinamic_optimizer':
            if self.step_count == 0:
                ## we generate the values to the goal
                self.q_value = self.dinamic_value_generator()
                rotation, movement = self.construct_optimal_path(sensors)
            else:    
                ## cosntruct the optimal path using the information obteined by a_star. This information about the maze is in self.bin_tile
                rotation, movement = self.construct_optimal_path(sensors)
        
        elif self.model == 'a_star_dinamic_a_star':
            if self.step_count == 0:
                ## we generate the values to the goal
                self.q_value = self.dinamic_value_generator()
                rotation, movement = self.a_star(sensors, heuristic_grid=self.q_value)
            else:    
                rotation, movement = self.a_star(sensors, heuristic_grid=self.q_value)
                
        elif self.model == 'random':
            rotation, movement = self.random_search(sensors)            
        
        elif self.model == 'random_detector':
            rotation, movement = self.random_search(sensors, d_end_detector=True)            
        
        elif self.model == 'random_detector_visited':
            rotation, movement = self.random_search(sensors, d_end_detector=True, visited_count=True)           
        else:
            print('Do not match any model name')
            
            return 0, 0 
        self.step_count +=1   
        if self.check_goal():
            self.save_goal()
            print 'Found goal in location %s in step %s'% (self.goal_location_exploit, self.found_goal_step_exploit)
       
        
        
        return rotation, movement
        
        
        
    def verbo(self, say_var):
        if self.verbose ==True:
                print say_var
        else:
            pass
        return None
                
        
        
        
        
        
        
        
        
        
            
        
        
        
        
        
        
