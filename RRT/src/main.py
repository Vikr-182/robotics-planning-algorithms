import pygame
import numpy
from os import system
from world import *
from robot import*
import time

clock = pygame.time.Clock()
np.random.seed(100)
h=600
w=600
gameDisplay=pygame.display.set_mode((h,w))
pygame.display.set_caption("Title")
gameDisplay.blit(pygame.transform.flip(gameDisplay, False, True), dest=(0, 0))

crashed = False; done = False

sim = World(h,w)

robo = RobotNonHolonomic_differential_drive(sim.spawn, sim.goal)
# robo = RobotNonHolonomic_tricycle_drive(sim.spawn, sim.goal)
# robo = RobotHolonomic(sim.spawn, sim.goal)
robot_size = robo.robot_size + 3

obstacles = [[0,0,100,100],
             [100,150,10,100],
             [250,250,10,300],
             [100, 245, 150,10],
             [350, 0, 10, 300],
             [350, 295, 170, 10],
             [250, 515, 300, 10],
             [450, 325, 10, 140],
             [350, 395, 200, 10],
             [220, 50, 10, 150],
             [145, 125, 150, 10]]

sim.add_obstacles(obstacles)
sim.grow_obstacles(robot_size)


num_steps = 100000
step = 0
while not (crashed):
    print(step)
    step = step+1
    # time.sleep(1)

    # crashing the simulation manually
    for i in pygame.event.get():
      if i.type == pygame.KEYDOWN:
        if i.unicode == "q":
          crashed = True
    
    if step%5 == 0:
      goal = sim.goal      
    else:
      goalx = np.random.uniform(0,h)
      goaly = np.random.uniform(0,w)
      goal = (goalx, goaly)
    
    done = robo.RRT_step(goal, sim)

    gameDisplay.fill((255,255,255))
    sim.print_obstacles(gameDisplay, robot_size)
    robo.print_paths(gameDisplay, type='center', done=done)
    # robo.print_paths(gameDisplay, type='wheels', done=done)
    pygame.display.update()
    clock.tick(60)

    # if step>num_steps:
    #   done = True
