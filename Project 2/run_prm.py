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
from prm import *

num_samples=200
# problem = "vrep"
problem = 'env0.txt'
_STEP_LENGTH_POLYGON = 5
_STEP_LENGTH_VREP = 0.1
_RADIUS = 25

np.random.seed(0)

#load problem
if(problem == "vrep"):
    environment = vrepWrapper.vrepWrapper()
    step_length=0.1
else:
    environment = PolygonEnvironment()
    environment.read_env('Project 2/' + problem)
    step_length=_STEP_LENGTH_POLYGON
print("Running...")
print(problem + " with " + str(num_samples) + " samples and step length " + str(step_length) + " and radius " + str(_RADIUS))


num_dimensions = len(environment.start)
start_time = time.time()
collision_func = environment.test_collisions
local_planner = StraightLinePlanner(step_length, environment.test_collisions)

prm = PRM(num_samples, local_planner, num_dimensions, environment.lims,
                 collision_func, _RADIUS, _STEP_LENGTH_POLYGON)

prm.build_prm()
plan, visited = prm.query(environment.start, environment.goal)
    debugThing = environment.draw_plan(plan, prm,False, False,True)

if(problem == "vrep"):
    environment.vrepReset()

run_time = time.time() - start_time
print('plan:', plan)
print('run_time =', run_time)



if(problem == "vrep"):
    time.sleep(10)
    environment.vrepStop()
