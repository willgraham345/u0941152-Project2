# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 12:17:29 2019

@author: tabor
"""
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
import vrepWrapper
from rrt import *

#Config parameters
connect = False
bidirection = True
num_samples=5000
problem = 'env0.txt'
# i.e. problem = 'env1_99_0_0.txt' for the first environment with a user-defined configuration
_CONNECT_PROB = 0.05
_STEP_LENGTH_POLYGON = 2
_STEP_LENGTH_VREP = 0.1

np.random.seed(0)

#load problem
if(problem == "vrep"):
    environment = vrepWrapper.vrepWrapper()
    step_length=0.1
else:
    environment = PolygonEnvironment()
    environment.read_env('Project 2/' + problem)
    step_length=_STEP_LENGTH_POLYGON
#output to screen to show program is running.
print("Running...")
print(problem + " with " + str(num_samples) + " samples and step length " + str(step_length) + " and connect prob " + str(_CONNECT_PROB))


dims = len(environment.start)
start_time = time.time()

rrt = RRT(num_samples,
          dims,
          step_length,
          lims = environment.lims,
          connect_prob = _CONNECT_PROB,
          collision_func=environment.test_collisions)
if connect:
    plan = rrt.build_rrt_connect(environment.start, environment.goal)
elif bidirection:
    plan = rrt.build_bidirectional_rrt_connect(environment.start, environment.goal)
else:
    plan = rrt.build_rrt(environment.start, environment.goal)

if(problem == "vrep"):
   environment.vrepReset()

run_time = time.time() - start_time
print('plan:', plan)
print('run_time =', run_time)

debugThing = environment.draw_plan(plan, rrt,False, False,True)

if(problem == "vrep"):
    time.sleep(10)
    environment.vrepStop()
