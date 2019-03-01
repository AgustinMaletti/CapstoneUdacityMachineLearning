# Capstone Udacity Machine Learning


This is my project capstone for Udacity nanodegree on Machine Learning.
The background behind this project is the robot mouse competition on solving mazes. In the first phase explore and in the second exploit.

This program apply the a star algorithm for exploring the maze.
Them apply technique of dynamic programming for calculating the value function.
Them create an optimal path following the policy.

Also you can use random models with differents features for benchmark purporses.

The project only require numpy

With the next commands you can execute the program.

1) python tester.py test_maze_01.py a_star_dinamic_optimizer
2) python tester.py test_maze_01.py random
3) python tester.py test_maze_01.py random_detector
4) python tester.py test_maze_01.py random_detector_visited

After the model you can also pass a number representing the required exploration percentage in the first phase. If nothing is passed the algorithmm of exploration will pass to the next phase as soon as found the goal.
