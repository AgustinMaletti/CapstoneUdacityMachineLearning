from maze27 import Maze
from robotpy27 import Robot
import sys

# global dictionaries for robot movement and sensing
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}

dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up':[0, 1], 'right':[1, 0],'down':[0, -1],'left': [-1, 0]}

dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

# test and score parameters
# max steps per run
## exploration and exploiting phase share the max_time
max_time = 1000
train_score_mult = 1/30.

if __name__ == '__main__':
    '''
    This script tests a robot based on the code in robot.py on a maze given
    as an argument when running the script.
    '''
    ## python tester.py[0][ test_maze01.txt[1] a_star[2] verbo[3] 60[4](explorationrate)
    # Create a maze based on input argument on command line.
    testmaze = Maze( str(sys.argv[1]) )
    ## catch the election of the model from the second argument: random or first_search or a_star ordinamic
    model_ = str(sys.argv[2])
    
    try:
        exploration_limit_ = int(sys.argv[3]) 
    
    except IndexError:
        exploration_limit_ = 0
   
    try:
        file_name = sys.argv[4]
    except IndexError:
        pass
        
    
    try:
        if sys.argv[5] == 'verbo':
            verbo_ = True
        else:
            pass    
    except IndexError:
        verbo_= False

   
        
    ## Intitialize a robot; robot receives info about maze dimensions. model options are: random first_search
    robot_mouse = Robot(testmaze.dim, model=model_, verbo= verbo_, exploration_limit=exploration_limit_)
    ## for practical reason cacth the limit of exploration from terminal command
    robot_mouse.max_time = max_time
    # im geting the list of bin walls number of the maze for debugging purpose
    robot_mouse.tester_bin_wall = testmaze.walls
    # Record robot performance over two runs.
    runtimes = []
    total_time = 0
    ## the robot make two run
    for run in range(2):
        print "Starting run %s." % (run)

        # Set the robot in the start position. Note that robot position
        # parameters are independent of the robot itself.
        robot_pos = {'location': [0, 0], 'heading': 'up'}

        run_active = True
        hit_goal = False
        
        ##While
        while run_active:
            ## for debugging purpose
            robot_mouse.tester_loc  = robot_pos['location']
            robot_mouse.step = total_time
            # check for end of time 
            # time are the steps
            total_time += 1
            if total_time > max_time:
                run_active = False
                print "Allotted time exceeded."
                break

            
            
            # provide robot with sensor information, get actions, heading is the direction variable of the function dist_to_walls
            ## for each step meke the sensing action and them move, each move could contain 3 moves
            sensing = [testmaze.dist_to_wall(robot_pos['location'], heading)
                       for heading in dir_sensors[robot_pos['heading']]]
            
            
            rotation, movement = robot_mouse.next_move(sensing)

           
            
            # check for a reset
            if (rotation, movement) == ('Reset', 'Reset'):
                if run == 0 and hit_goal:
                    run_active = False
                    runtimes.append(total_time)
                    print"Ending first run. Starting next run."
                    ## print('Found goal in exploration  in location{} in step {}'.format(robot_pos['location'], total_time))
                    ## if the robot 
                    break
                    
                elif run == 0 and not hit_goal:
                    print "Cannot reset - robot has not hit goal yet."
                    continue
                else:
                    print "Cannot reset on runs after the first."
                    continue
            ## First rotate them move
            # perform rotation
            if rotation == -90:
                ##  heading is equal to the left index in dir_sensors
                robot_pos['heading'] = dir_sensors[robot_pos['heading']][0]
            elif rotation == 90:
                ## heading is equal to the rigth index of the dir_sensors
                robot_pos['heading'] = dir_sensors[robot_pos['heading']][2]
            elif rotation == 0:
                pass
            else:
                print "Invalid rotation value, no rotation performed.", rotation

            # perform movement
            if abs(movement) > 3:
                print "Movement limited to three squares in a turn."
            movement = max(min(int(movement), 3), -3) # fix to range [-3, 3]
            
            while movement:
                if movement > 0:
                    if testmaze.is_permissible(robot_pos['location'], robot_pos['heading']):
                        ## robot_mouse.back_sense =1
                        robot_pos['location'][0] += dir_move[robot_pos['heading']][0]
                        robot_pos['location'][1] += dir_move[robot_pos['heading']][1]
                        movement -= 1
                    ## if not permissible    
                    else:
                        print "Movement stopped by wall."
                        movement = 0
                ## if the movement is negative        
                else:
                    rev_heading = dir_reverse[robot_pos['heading']]
                    if testmaze.is_permissible(robot_pos['location'], rev_heading):
                        robot_pos['location'][0] += dir_move[rev_heading][0]
                        robot_pos['location'][1] += dir_move[rev_heading][1]
                        movement += 1
                    ## if not permissible     
                    else:
                        print "Movement stopped by wall."
                        movement = 0

            # check for goal entered
            goal_bounds = [testmaze.dim/2 - 1, testmaze.dim/2]
            
            if robot_pos['location'][0] in goal_bounds and robot_pos['location'][1] in goal_bounds:
                hit_goal = True
                if run != 0:
                    runtimes.append(total_time - sum(runtimes))
                    run_active = False
                    print "\nGoal found; run %s completed! first run use %s steps, second run use %s steps " % (run, runtimes[0],runtimes[1])

    # Report score if robot is successful.
    if len(runtimes) == 2:
        print "\n\nTask complete! Score: %s \n\n" % (train_score_mult*runtimes[0] + runtimes[1] )
        score = train_score_mult*runtimes[0] + runtimes[1]
        ## robot_mouse.save_to_file(file_name, runtimes[0], score)
    robot_mouse.report()
    ## try:
    ## if file_name:
        
    ## except:
        ## pass    
    
    
    
